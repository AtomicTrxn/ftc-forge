#!/usr/bin/env python3
"""Repeatable URDF/STL preparation, versioned settings, review and CAD migration.
Only standard-library dependencies; source files are copied, never edited.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import struct
import tempfile
import uuid
import xml.etree.ElementTree as ET
import zipfile
from prepare_biobuzz import extract, clustered, bounds, write_stl, combine, serial_pose, IDENTITY
from prepare_rev_duo import rotation

VERSION = 1
RUNTIME_KEYS = ['drive', 'intake', 'tires', 'flexible_intake', 'torus_retention', 'servoPhysics',
                'collision_omissions', 'total_mass_kg', 'vhacd_max_hulls', 'start_height_m', 'imu_latency_ms']
DEFAULTS = dict(length_unit='m', up_axis='Z', origin_xyz_m=[0.,0.,0.], origin_rpy_rad=[0.,0.,0.],
                collision_margin_m=.002, floor_thickness_m=.02, floor_margin_m=.001, contact_stiffness_n_per_m=1e30, contact_damping_ns_per_m=.1, chassis_lock_level=True, use_contact_compliance=False, fixed_restitution=0., fixed_rolling_friction=0., fixed_spinning_friction=0., floor_strategy='source', visual_grid_m=.0005, collision_grid_m=.001,
                max_hulls=8, fallback_mass_kg=1., minimum_thickness_m=.001,
                friction=.6, restitution=.15, rolling_friction=.005, spinning_friction=.005,
                field_half_extents_m=[1.8288,1.8288], floor_top_m=0.)


def canonical(value): return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
def digest(value): return hashlib.sha256(canonical(value).encode()).hexdigest()
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def uid(): return str(uuid.uuid4())
def pointer(name):return name.replace('~','~0').replace('/','~1')
def read(path): return json.loads(Path(path).read_text())
def atomic_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix='.save-',dir=path.parent)
    try:
        with os.fdopen(fd,'w') as out:
            json.dump(value,out,indent=2,allow_nan=False);out.write('\n');out.flush();os.fsync(out.fileno())
        os.replace(tmp,path)
    finally:
        if Path(tmp).exists():Path(tmp).unlink()


def xml(path):
    data=Path(path).read_bytes()
    if len(data)>20_000_000 or b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper():raise ValueError('Unsupported XML budget/DTD/entities')
    root=ET.fromstring(data)
    if root.tag!='robot':raise ValueError('Expected URDF robot')
    return root


def vec(text, size=3):
    values=[float(x) for x in text.split()]
    if len(values)!=size or not all(math.isfinite(x) for x in values):raise ValueError('Invalid finite vector')
    return values


def finite(value, low=None, high=None):
    if not isinstance(value,(float,int)) or isinstance(value,bool) or not math.isfinite(value) or low is not None and value<low or high is not None and value>high:raise ValueError(f'Invalid parameter value: {value}')
    return float(value)


def triangles(path):
    data=Path(path).read_bytes()
    if len(data)>=84:
        count=struct.unpack_from('<I',data,80)[0]
        if 0<count<=1_000_000 and len(data)==84+50*count:
            result=[tuple(tuple(row[3+i*3:6+i*3]) for i in range(3)) for row in struct.iter_unpack('<12fH',data[84:])]
        else:result=None
    else:result=None
    if result is None:
        points=[]
        for line in data.decode('ascii').splitlines():
            if line.strip().startswith('vertex '):points.append(tuple(vec(line.strip()[7:])))
        if not points or len(points)%3 or len(points)>3_000_000:raise ValueError('Invalid STL triangle budget')
        result=[tuple(points[i:i+3]) for i in range(0,len(points),3)]
    if any(not all(math.isfinite(x) for v in t for x in v) for t in result):raise ValueError('Nonfinite STL')
    return result


def resolve_mesh(urdf, ref, trusted=False):
    root=urdf.parent.parent if urdf.parent.name=='urdf' else urdf.parent
    if ref.startswith('package://'):
        part=PurePosixPath(ref[10:]);relative=PurePosixPath(*part.parts[1:])
        if '..' in relative.parts:raise ValueError('Mesh escapes package')
        candidates=[root/str(relative),urdf.parent/str(relative)]
        path=next((p for p in candidates if p.is_file()),candidates[0])
    else:path=urdf.parent/ref
    path=path.resolve()
    if not trusted and not path.is_relative_to(root.resolve()):raise ValueError('Mesh escapes extracted source package')
    if path.suffix.lower()!='.stl' or not path.is_file():raise ValueError(f'Missing STL: {ref}')
    return path


def source_copy(urdf, destination, trusted=False):
    root=xml(urdf);destination.mkdir(parents=True,exist_ok=True)
    cache={}
    for mesh in root.findall('.//mesh'):
        path=resolve_mesh(urdf,mesh.get('filename',''),trusted);key=cache.get(('hash',path)) or sha(path);cache[('hash',path)]=key
        if key not in cache:
            target=destination/'assets'/f'{key}.stl';target.parent.mkdir(exist_ok=True);shutil.copyfile(path,target);cache[key]=str(target.relative_to(destination))
        mesh.set('filename',cache[key])
    ET.indent(root);ET.ElementTree(root).write(destination/'model.urdf',encoding='utf-8',xml_declaration=True)
    return destination/'model.urdf'


def geometry(g, source_dir, mesh_cache):
    element=next(iter(g));kind=element.tag
    data={'kind':kind}
    if kind=='mesh':
        file=element.get('filename');path=source_dir/file
        if file not in mesh_cache:mesh_cache[file]=(bounds(triangles(path)),sha(path))
        scale=vec(element.get('scale','1 1 1'));bb,checksum=mesh_cache[file]
        data.update(file=file,sha256=checksum,scale=scale,bounds=[[min(bb[0][i]*scale[i],bb[1][i]*scale[i]) for i in range(3)],[max(bb[0][i]*scale[i],bb[1][i]*scale[i]) for i in range(3)]])
    elif kind=='box':
        sizes=vec(element.get('size'));data.update(size=sizes,bounds=[[-x/2 for x in sizes],[x/2 for x in sizes]])
    elif kind=='sphere':
        r=float(element.get('radius'));data.update(radius=r,bounds=[[-r]*3,[r]*3])
    elif kind=='cylinder':
        r,h=float(element.get('radius')),float(element.get('length'));data.update(radius=r,length=h,bounds=[[-r,-r,-h/2],[r,r,h/2]])
    else:raise ValueError('Unsupported geometry: '+kind)
    for key in ['radius','length']:
        if key in data:finite(data[key],1e-12,1000000)
    if kind=='box':
        for v in data['size']:finite(v,1e-12,1000000)
    return data


def origin(el):
    o=el.find('origin');return {'xyz':vec(o.get('xyz','0 0 0')),'rpy':vec(o.get('rpy','0 0 0'))} if o is not None else {'xyz':[0.,0.,0.],'rpy':[0.,0.,0.]}


def describe(urdf):
    root=xml(urdf);links={};mesh_cache={};parents={};joints={}
    names=[l.get('name') for l in root.findall('link')];joint_names=[j.get('name') for j in root.findall('joint')]
    if not names or len(names)>2000 or len(set(names))!=len(names) or any(not n for n in names) or len(set(joint_names))!=len(joint_names):raise ValueError('Unique named links/joints required within 2000-link budget')
    for j in root.findall('joint'):
        child=j.find('child').get('link');parent=j.find('parent').get('link')
        if child in parents:raise ValueError('Multiple URDF parents')
        parents[child]=parent;joints[child]={'name':j.get('name'),'type':j.get('type'),'parent':parent,**origin(j),'axis':vec(j.find('axis').get('xyz')) if j.find('axis') is not None else [1.,0.,0.], 'lower':float(j.find('limit').get('lower','0')) if j.find('limit') is not None else 0.,'upper':float(j.find('limit').get('upper','0')) if j.find('limit') is not None else 0.}
    for link in root.findall('link'):
        name=link.get('name');visuals=[];collisions=[]
        for tag,result in [('visual',visuals),('collision',collisions)]:
            for v in link.findall(tag):result.append({**origin(v),'geometry':geometry(v.find('geometry'),urdf.parent,mesh_cache),'material':ET.tostring(v.find('material'),encoding='unicode').strip() if v.find('material') is not None else None})
        points=[]
        for v in visuals or collisions:
            bb=v['geometry']['bounds'];r=rotation(*v['rpy'])
            points.extend(combine((r,v['xyz']),(IDENTITY[0],[x,y,z]))[1] for x in [bb[0][0],bb[1][0]] for y in [bb[0][1],bb[1][1]] for z in [bb[0][2],bb[1][2]])
        bb=[[f(p[i] for p in points) for i in range(3)] for f in (min,max)] if points else [[0.,0.,0.],[0.,0.,0.]]
        inertial=link.find('inertial');mass=float(inertial.find('mass').get('value')) if inertial is not None else 0.
        it=inertial.find('inertia') if inertial is not None else None
        links[name]={'name':name,'parent':parents.get(name),'joint':joints.get(name),'visuals':visuals,'collisions':collisions,'bounds':bb,'mass_kg':mass,'inertia':dict(it.attrib) if it is not None else None,'inertial_origin':origin(inertial) if inertial is not None else None}
        links[name]['geometry_signature']=digest([v['geometry'] for v in visuals or collisions])
    roots=set(links)-set(parents)
    if len(roots)!=1 or not set(parents)<=set(links) or not set(parents.values())<=set(links):raise ValueError('Expected one rooted URDF tree')
    top=next(iter(roots));reached=set()
    def visit(name):
        if name in reached:raise ValueError('Cycle in URDF')
        reached.add(name)
        for child,parent in parents.items():
            if parent==name:visit(child)
    visit(top)
    if reached!=set(links):raise ValueError('Disconnected URDF')
    return {'root':top,'links':links,'transmission_bindings':[{'joint':e.find('joint').get('name'),'actuators':[{'name':a.get('name'),'mechanicalReduction':float(a.findtext('mechanicalReduction','1'))} for a in e.findall('actuator')]} for e in root.findall('transmission')],'fingerprint':digest({'links':links,'materials':[ET.tostring(e,encoding='unicode').strip() for e in root.findall('material')],'transmissions':[ET.tostring(e,encoding='unicode').strip() for e in root.findall('transmission')]})}


def body_owners(source, runtime=None):
    owners={};links=source['links'];runtime=runtime or {}
    drive=runtime.get('drive',{});motors={drive.get('left_motor'),drive.get('right_motor')} if drive else {'left_front_drive','right_front_drive','left_back_drive','right_back_drive'}
    wheel_joints={t['joint'] for t in source.get('transmission_bindings',[]) if any(a['name'] in motors for a in t['actuators'])};wheel_branches=set()
    def owner(name):
        if name not in owners:
            link=links[name];parent=link['parent']
            if parent is None or link.get('role')=='piece':owners[name]=name
            else:
                powner=owner(parent);wheel=parent in wheel_branches or link['joint']['name'] in wheel_joints
                if wheel:wheel_branches.add(name)
                owners[name]=powner if link['joint']['type']=='fixed' or wheel else name
        return owners[name]
    for name in links:owner(name)
    return owners


def new_profile(urdf, kind, runtime=None):
    description=describe(urdf);source_xml=xml(urdf);owners=body_owners(description,runtime)
    covered={owners[n] for n,l in description['links'].items() if l['collisions']}
    p={'schema_version':VERSION,'profile_id':uid(),'revision_id':uid(),'model_kind':kind,'name':xml(urdf).get('name','Imported model'),'adapter':'general','source':{**description,'files':{str(f.relative_to(urdf.parent)):sha(f) for f in urdf.parent.rglob('*') if f.is_file()}},'parameters':copy.deepcopy(DEFAULTS),'entities':{},'runtime':runtime or {},'provenance':{},'review':{'state':'draft'},'migration':{'pending':[],'decisions':[]}}
    for n,l in description['links'].items():
        lo,hi=l['bounds'];size=[max(DEFAULTS['minimum_thickness_m'],hi[i]-lo[i]) for i in range(3)]
        strategy='source' if l['collisions'] else 'none' if owners[n] in covered or not l['visuals'] else 'visual' if all(v['geometry']['kind']!='mesh' for v in l['visuals']) else 'box'
        p['entities'][n]={'id':uid(),'settings':{'role':'structure','collision_strategy':strategy,'box_size_m':size,'collision_xyz_m':[(lo[i]+hi[i])/2 for i in range(3)],'collision_rpy_rad':[0.,0.,0.],'sphere_radius_m':max(size)/2,'cylinder_length_m':size[2],'mass_mode':'source' if l['mass_kg']>0 else 'fallback','mass_kg':l['mass_kg'] if l['mass_kg']>0 else DEFAULTS['fallback_mass_kg'],'inertia_mode':'source' if l['inertia'] and l['mass_kg']>0 else 'box','inertia_kg_m2':[.01,.01,.01,0.,0.,0.],'com_xyz_m':[0.,0.,0.],'material_override':False,'friction':.6,'restitution':.15,'rolling_friction':.005,'spinning_friction':.005,'joint':copy.deepcopy(l['joint']),'joint_spring_nm_per_rad':0.,'joint_damping_nm_s':0.,'joint_rest_rad':0.,'joint_spring_n_per_m':0.,'joint_damping_ns_per_m':0.,'joint_rest_m':0.,'actuators':[{'name':a.get('name'),'mechanicalReduction':float(a.findtext('mechanicalReduction','1'))} for tx in source_xml.findall('transmission') if tx.find('joint').get('name')==(l['joint'] or {}).get('name') for a in tx.findall('actuator')]},'assumptions':['Bounding box may fill hollow openings; review shape strategy'] if strategy=='box' else ['Fixed-body aggregate proxy; surface coverage requires review'] if strategy=='none' and l['visuals'] else [],'provenance':'source CAD' if strategy=='source' else 'generated default'}
    if kind=='field':
        candidates=[(math.prod([l['bounds'][1][i]-l['bounds'][0][i] for i in range(2)]),n) for n,l in description['links'].items() if l['visuals'] and l['bounds'][1][2]-l['bounds'][0][2]<.05]
        if not candidates:
            p['parameters']['floor_strategy']='generated';p['entities'][description['root']]['assumptions'].append('No surface inferred: generated floor uses editable extents, height and thickness')
        if candidates:
            _,floor=max(candidates);p['entities'][floor]['settings']['role']='surface'
            lo,hi=description['links'][floor]['bounds'];p['parameters']['field_half_extents_m']=[max(.1,(hi[i]-lo[i])/2) for i in range(2)]
            p['entities'][floor]['assumptions'].append('Surface and field extents inferred; verify field origin and playable boundary')
    for n,e in p['entities'].items():
        if description['links'][n]['mass_kg']<=0 and description['links'][n]['visuals']:e['assumptions'].append('Missing source mass: explicit fallback mass and box inertia; replace with measured values')
        for key in e['settings']:p['provenance']['entities/'+pointer(n)+'/settings/'+key]='source CAD' if key in ['joint','actuators'] or key=='mass_kg' and description['links'][n]['mass_kg']>0 else 'generated default'
    for k in p['parameters']:p['provenance']['parameters/'+k]='generated default'
    for k in p['runtime']:p['provenance']['runtime/'+k]='user supplied (captured existing project)'
    return p


def effective(p):return {k:p[k] for k in ['schema_version','model_kind','adapter','source','parameters','entities','runtime','migration']}
def effective_digest(p):return digest(effective(p))


def validate(p):
    if p.get('schema_version')!=VERSION or p.get('model_kind') not in ['robot','field']:raise ValueError('Unsupported model profile version/type')
    canonical(p);par=p['parameters']
    if par['length_unit'] not in ['m','mm','in'] or par['up_axis'] not in ['Z','Y']:raise ValueError('Select length units and up axis')
    if par['floor_strategy'] not in ['source','generated']:raise ValueError('Choose source or generated floor')
    finite(par['floor_thickness_m'],.001,.5);finite(par['floor_margin_m'],0,.02);finite(par['floor_top_m'],-100,100);finite(par['contact_stiffness_n_per_m'],.001,1e30);finite(par['contact_damping_ns_per_m'],0,10000);finite(par['collision_margin_m'],0,.02);finite(par['visual_grid_m'],.00001,.001);finite(par['collision_grid_m'],.00001,.01);finite(par['fallback_mass_kg'],.000001,10000);finite(par['minimum_thickness_m'],.000001,.1)
    if int(par['max_hulls'])!=par['max_hulls'] or not 1<=par['max_hulls']<=16:raise ValueError('max_hulls must be 1..16')
    for key,high in [('friction',2),('restitution',1),('rolling_friction',2),('spinning_friction',2),('fixed_restitution',1),('fixed_rolling_friction',2),('fixed_spinning_friction',2)]:finite(par[key],0,high)
    for v in par['field_half_extents_m']:finite(v,.01,100)
    for key in ['origin_xyz_m','origin_rpy_rad']:
        if len(par[key])!=3:raise ValueError('Expected three coordinates')
        for v in par[key]:finite(v,-1000,1000)
    for name,e in p['entities'].items():
        if name not in p['source']['links']:raise ValueError('Unknown entity mapping')
        s=e['settings']
        if s['role'] not in ['structure','surface','barrier','apparatus','piece','decoration','reference']:raise ValueError('Invalid role')
        if s['collision_strategy'] not in ['source','visual','box','sphere','cylinder','mesh','none']:raise ValueError('Invalid collision strategy')
        for v in s['box_size_m']:finite(v,.000001,1000)
        finite(s.get('sphere_radius_m',max(s['box_size_m'])/2),1e-12,1000);finite(s.get('cylinder_length_m',s['box_size_m'][2]),1e-12,1000)
        finite(s['mass_kg'],1e-12,10000)
        for key in ['collision_xyz_m','collision_rpy_rad','com_xyz_m']:
            if len(s[key])!=3:raise ValueError('Expected three coordinates')
            for v in s[key]:finite(v,-1000,1000)
        for a in s['actuators']:
            if not isinstance(a['name'],str) or not a['name']:raise ValueError('Actuator name required')
            finite(a['mechanicalReduction'],-100000,100000)
            if abs(a['mechanicalReduction'])<1e-6:raise ValueError('Mechanical reduction must be nonzero')
        for key in ['joint_spring_nm_per_rad','joint_damping_nm_s']:finite(s[key],0,10000)
        finite(s['joint_rest_rad'],-math.pi,math.pi)
        for key in ['joint_spring_n_per_m','joint_damping_ns_per_m']:finite(s.get(key,0),0,10000)
        finite(s.get('joint_rest_m',0),-1000,1000)
        j=s['joint']
        if j:
            if j['type'] not in ['fixed','revolute','continuous','prismatic']:raise ValueError('Unsupported joint type')
            for key in ['xyz','rpy','axis']:
                if len(j[key])!=3:raise ValueError('Expected joint vector')
                for v in j[key]:finite(v,-1000,1000)
            if j['type']!='fixed' and sum(x*x for x in j['axis'])<1e-12:raise ValueError('Joint axis must be nonzero')
            if j['type'] in ['revolute','prismatic'] and j['lower']>=j['upper']:raise ValueError('Joint limits must be ordered')
        for key,high in [('friction',2),('restitution',1),('rolling_friction',2),('spinning_friction',2)]:finite(s[key],0,high)
        if s['mass_mode'] not in ['source','override','fallback'] or s['inertia_mode'] not in ['source','box','override']:raise ValueError('Invalid mass/inertia mode')
        if s['inertia_mode']=='override':
            xx,yy,zz,xy,xz,yz=s['inertia_kg_m2']
            if xx<=0 or xx*yy-xy*xy<=0 or xx*yy*zz+2*xy*xz*yz-xx*yz*yz-yy*xz*xz-zz*xy*xy<=0:raise ValueError('Inertia tensor must be positive definite')


def scale_source(root, scale):
    for o in root.findall('.//origin'):o.set('xyz',' '.join(str(x*scale) for x in vec(o.get('xyz','0 0 0'))))
    for g in root.findall('.//geometry'):
        el=next(iter(g))
        if el.tag=='mesh':el.set('scale',' '.join(str(x*scale) for x in vec(el.get('scale','1 1 1'))))
        elif el.tag=='box':el.set('size',' '.join(str(x*scale) for x in vec(el.get('size'))))
        else:
            for key in ['radius','length']:
                if key in el.attrib:el.set(key,str(float(el.get(key))*scale))
    for i in root.findall('.//inertia'):
        for k,v in list(i.attrib.items()):i.set(k,str(float(v)*scale*scale))
    for j in root.findall('joint'):
        if j.get('type')=='prismatic' and j.find('limit') is not None:
            for k in ['lower','upper']:j.find('limit').set(k,str(float(j.find('limit').get(k))*scale))


def add_shape(link, shape, position, angles=(0,0,0)):
    c=ET.SubElement(link,'collision');ET.SubElement(c,'origin',xyz=' '.join(map(str,position)),rpy=' '.join(map(str,angles)));ET.SubElement(c,'geometry').append(copy.deepcopy(shape))


def compiled_robot(p, source_dir, output):
    validate(p);root=xml(source_dir/'model.urdf');par=p['parameters'];scale={'m':1.,'mm':.001,'in':.0254}[par['length_unit']];scale_source(root,scale)
    links={l.get('name'):l for l in root.findall('link')};cache={};asset_cache=read(output/'mesh-cache.json') if (output/'mesh-cache.json').is_file() else {}
    for name,l in links.items():
        s=p['entities'][name]['settings'];mode=s['collision_strategy']
        if mode!='source':
            for c in l.findall('collision'):l.remove(c)
            if mode=='box':add_shape(l,ET.Element('box',size=' '.join(map(str,s['box_size_m']))),s['collision_xyz_m'],s['collision_rpy_rad'])
            elif mode in ['sphere','cylinder']:
                shape=ET.Element(mode,radius=str(s.get('sphere_radius_m',max(s['box_size_m'])/2)))
                if mode=='cylinder':shape.set('length',str(s.get('cylinder_length_m',s['box_size_m'][2])))
                add_shape(l,shape,s['collision_xyz_m'],s['collision_rpy_rad'])
            elif mode in ['visual','mesh']:
                for v in l.findall('visual'):
                    g=next(iter(v.find('geometry')))
                    if mode=='mesh' and g.tag!='mesh':continue
                    o=origin(v);add_shape(l,g,o['xyz'],o['rpy'])
        if s['role'] in ['decoration','reference']:
            for c in l.findall('collision'):l.remove(c)
        inertial=l.find('inertial')
        if inertial is None:inertial=ET.SubElement(l,'inertial');ET.SubElement(inertial,'mass',value='0');ET.SubElement(inertial,'inertia',**{k:'0' for k in ['ixx','iyy','izz','ixy','ixz','iyz']})
        mass=inertial.find('mass');old=float(mass.get('value'));new=old if s['mass_mode']=='source' else par['fallback_mass_kg'] if s['mass_mode']=='fallback' else s['mass_kg']
        if not l.findall('visual') and not l.findall('collision') and old==0:new=0.
        mass.set('value',str(new));tensor=inertial.find('inertia')
        if s['inertia_mode']=='source' and old>0:
            for k,v in list(tensor.attrib.items()):tensor.set(k,str(float(v)*new/old))
        else:
            x,y,z=s['box_size_m'];values=[new*(y*y+z*z)/12,new*(x*x+z*z)/12,new*(x*x+y*y)/12,0,0,0] if s['inertia_mode']=='box' else s['inertia_kg_m2']
            for k,v in zip(['ixx','iyy','izz','ixy','ixz','iyz'],values):tensor.set(k,str(v))
            o=inertial.find('origin')
            if o is None:o=ET.SubElement(inertial,'origin')
            o.set('xyz',' '.join(map(str,s['com_xyz_m'])));o.set('rpy','0 0 0')
    for j in root.findall('joint'):
        child=j.find('child').get('link');settings=p['entities'][child]['settings']['joint']
        if settings is not None:
            j.set('type',settings['type']);j.find('parent').set('link',settings['parent'])
            o=j.find('origin')
            if o is None:o=ET.SubElement(j,'origin')
            o.set('xyz',' '.join(str(x*scale) for x in settings['xyz']));o.set('rpy',' '.join(map(str,settings['rpy'])))
            a=j.find('axis')
            if a is None:a=ET.SubElement(j,'axis')
            a.set('xyz',' '.join(map(str,settings['axis'])))
            if settings['type'] in ['revolute','prismatic']:
                lim=j.find('limit')
                if lim is None:lim=ET.SubElement(j,'limit')
                for k in ['lower','upper']:lim.set(k,str(settings[k]*(scale if settings['type']=='prismatic' else 1)))
    # Bindings are editable independently of source joint names.
    for tx in root.findall('transmission'):root.remove(tx)
    for name,e in p['entities'].items():
        settings=e['settings'];joint=settings['joint']
        if settings['actuators'] and joint:
            tx=ET.SubElement(root,'transmission',name='forge_'+joint['name']);ET.SubElement(tx,'joint',name=joint['name'])
            for a in settings['actuators']:
                act=ET.SubElement(tx,'actuator',name=a['name']);ET.SubElement(act,'mechanicalReduction').text=str(a['mechanicalReduction'])
    # Collision and visual assets are separate; visual simplification cannot alter contact meshes.
    for tag,grid,budget in [('visual',par['visual_grid_m'],100_000),('collision',par['collision_grid_m'],200_000)]:
        for v in root.findall('.//'+tag):
            for m in v.findall('.//mesh'):
                file=m.get('filename');key=(file,tag)
                if key not in cache:
                    cache_id=digest([file,tag,grid,scale]);target=output/'assets'/(Path(file).stem+'-'+tag+'.stl');target.parent.mkdir(exist_ok=True)
                    if not target.is_file() or asset_cache.get(cache_id)!=sha(target):
                        original=triangles(source_dir/file);reduced=clustered(original,grid/scale) if len(original)>10_000 else original
                        if not reduced:reduced=original
                        if len(reduced)>budget:raise ValueError('Mesh exceeds prepared budget; increase simplification tolerance or choose a primitive')
                        write_stl(target,reduced);asset_cache[cache_id]=sha(target)
                    cache[key]=str(target.relative_to(output))
                m.set('filename',cache[key])
    atomic_json(output/'mesh-cache.json',asset_cache)
    # Explicit source basis/origin transform, independent of camera framing.
    angles=par['origin_rpy_rad'][:]
    if par['up_axis']=='Y':angles[0]+=math.pi/2
    if any(par['origin_xyz_m']) or any(angles):
        old=p['source']['root'];new='forge_origin_'+p['profile_id'][:8]
        ET.SubElement(root,'link',name=new);j=ET.SubElement(root,'joint',name=new+'_placement',type='fixed');ET.SubElement(j,'parent',link=new);ET.SubElement(j,'child',link=old);ET.SubElement(j,'origin',xyz=' '.join(map(str,par['origin_xyz_m'])),rpy=' '.join(map(str,angles)))
    return root


def world_poses(root):
    links={l.get('name') for l in root.findall('link')};joints=root.findall('joint');parents={j.find('child').get('link') for j in joints};poses={next(iter(links-parents)):IDENTITY}
    for _ in range(len(links)):
        for j in joints:
            a,b=j.find('parent').get('link'),j.find('child').get('link')
            if a in poses and b not in poses:
                o=origin(j);poses[b]=combine(poses[a],(rotation(*o['rpy']),o['xyz']))
    if set(poses)!=links:raise ValueError('Edited joints do not form a rooted tree')
    return poses


def change_units(path, unit):
    p=read(path);factors={'m':1.,'mm':.001,'in':.0254}
    ratio=factors[unit]/factors[p['parameters']['length_unit']]
    if p['model_kind']=='field' and p['parameters']['floor_strategy']=='source' and p['provenance'].get('parameters/field_half_extents_m')=='generated default':p['parameters']['field_half_extents_m']=[x*ratio for x in p['parameters']['field_half_extents_m']]
    for name,e in p['entities'].items():
        for key in ['box_size_m','collision_xyz_m','sphere_radius_m','cylinder_length_m']:
            if key not in e['settings']:continue
            base='entities/'+pointer(name)+'/settings/'+key;value=e['settings'][key]
            if isinstance(value,list):e['settings'][key]=[x if p['provenance'].get(base+'/'+str(i))=='user supplied' else x*ratio for i,x in enumerate(value)]
            elif p['provenance'].get(base)!='user supplied':e['settings'][key]=value*ratio
    p['parameters']['length_unit']=unit;p['provenance']['parameters/length_unit']='user supplied';atomic_json(path,p);compile_profile(path)


def compile_profile(path):
    path=Path(path).resolve();
    if 'revisions' in path.parts and not path.parent.name.startswith('.saving-'):raise ValueError('Saved revisions are immutable; open as an editable draft first')
    p=read(path);validate(p);folder=path.parent;source_dir=folder/'source';output=folder/'prepared';output.mkdir(exist_ok=True)
    for name,checksum in p['source']['files'].items():
        f=(source_dir/name).resolve()
        if not f.is_relative_to(source_dir.resolve()) or not f.is_file() or sha(f)!=checksum:raise ValueError('Source CAD changed; import and migrate it first')
    if p['adapter']=='biobuzz':
        manifest=copy.deepcopy(p['runtime']['field_manifest'])
        for mesh in manifest['meshes'].values():
            src=folder/mesh['file'];
            if mesh.get('prepared_sha256')!=sha(src):raise ValueError('Captured field mesh changed; recapture or migrate CAD')
            target=output/'assets'/Path(mesh['file']).name;target.parent.mkdir(exist_ok=True);shutil.copyfile(src,target);mesh['file']=str(target.relative_to(output))
        shutil.copytree(source_dir,output/'source',dirs_exist_ok=True);manifest['urdf']='source/model.urdf';manifest['model_parameters']=p['parameters'];manifest['field_half_extents_m']=p['parameters']['field_half_extents_m']
        atomic_json(output/'field.json',manifest);artifacts={'field':'prepared/field.json'}
    else:
        root=compiled_robot(p,source_dir,output);world_poses(root)
        artifacts={'robot':'prepared/robot.urdf'}
        if p['model_kind']=='field':
            poses=world_poses(root);pieces=[]
            joints=root.findall('joint');children={j.find('child').get('link'):j for j in joints};links={l.get('name'):l for l in root.findall('link')}
            selected=[n for n,e in p['entities'].items() if e['settings']['role']=='piece']
            for name in selected:
                if name not in children:raise ValueError('Field root cannot be a game piece')
                subtree=set()
                def descendants(n):
                    subtree.add(n)
                    for j in joints:
                        if j.find('parent').get('link')==n:descendants(j.find('child').get('link'))
                descendants(name)
                if any(n in selected for n in subtree if n!=name):raise ValueError('Select the piece assembly root, not nested pieces')
                piece=ET.Element('robot',name=name)
                for n in subtree:piece.append(copy.deepcopy(links[n]))
                for j in joints:
                    if j.find('parent').get('link') in subtree:piece.append(copy.deepcopy(j))
                for m in root.findall('material'):piece.append(copy.deepcopy(m))
                file='piece-'+p['entities'][name]['id']+'.urdf';ET.indent(piece);ET.ElementTree(piece).write(output/file,encoding='utf-8',xml_declaration=True)
                pieces.append({'id':p['entities'][name]['id'],'type':name,'file':'prepared/'+file,**serial_pose(poses[name])})
                for n in subtree:root.remove(links[n])
                for j in joints:
                    if j.find('child').get('link') in subtree and j in list(root):root.remove(j)
            artifacts={'field':'prepared/field.urdf','pieces':pieces}
        file=output/('field.urdf' if p['model_kind']=='field' else 'robot.urdf');ET.indent(root);ET.ElementTree(root).write(file,encoding='utf-8',xml_declaration=True)
    runtime=copy.deepcopy(p['runtime']);
    edited={'links':{n:{'parent':e['settings']['joint']['parent'] if e['settings']['joint'] else None,'joint':e['settings']['joint'],'role':e['settings']['role']} for n,e in p['entities'].items()},'transmission_bindings':[{'joint':e['settings']['joint']['name'],'actuators':e['settings']['actuators']} for e in p['entities'].values() if e['settings']['joint']]}
    omissions=runtime.setdefault('collision_omissions',{});owners=body_owners(edited,p['runtime'])
    for owner in set(owners.values()):
        group=[p['entities'][n]['settings'] for n,o in owners.items() if o==owner]
        if all(s['role'] in ['decoration','reference'] for s in group):omissions[owner]='User-declared nonphysical decoration/reference body'
    runtime.pop('field_manifest',None);runtime['vhacd_max_hulls']=int(p['parameters']['max_hulls']);runtime['collision_margin_m']=p['parameters']['collision_margin_m'];runtime['passive_joints']={e['settings']['joint']['name']:{k:e['settings'].get(k,0.) for k in ['joint_spring_nm_per_rad','joint_damping_nm_s','joint_rest_rad','joint_spring_n_per_m','joint_damping_ns_per_m','joint_rest_m']} for e in p['entities'].values() if e['settings']['joint']};runtime['model_materials']={n:{k:e['settings'][k] if e['settings']['material_override'] else p['parameters'][k] for k in ['friction','restitution','rolling_friction','spinning_friction']} for n,e in p['entities'].items()}
    if 'calibration_data' in runtime:
        atomic_json(output/'calibration.json',runtime.pop('calibration_data'));runtime['calibration']='prepared/calibration.json'
    for n,e in p['entities'].items():
        if e['settings']['material_override'] and owners[n]!=n:
            raise ValueError('Native materials apply per rigid body. Set the override on body '+owners[n]+' rather than fixed child '+n)
    ed=effective_digest(p);p['review']['state']='ready' if p['review'].get('digest')==ed and not p['migration']['pending'] else 'draft';atomic_json(path,p)
    files={str(f.relative_to(folder)):sha(f) for directory in [source_dir,output,folder/'legacy-assets'] for f in directory.rglob('*') if f.is_file()}
    receipt={'schema_version':VERSION,'effective_digest':ed,'profile_sha256':sha(path),'ready':p['review']['state']=='ready','files':files,'artifacts':artifacts,'runtime':runtime,'body_grouping':owners}
    atomic_json(folder/'receipt.json',receipt);return receipt


def xml_source_transmissions(p):return p['source'].get('transmission_bindings',[])

def migrate_joint(settings, previous, current):
    if current is None:return None
    result=copy.deepcopy(current)
    if settings and previous:
        for key,value in settings.items():
            if key!='name' and value!=previous.get(key):result[key]=copy.deepcopy(value)
    return result


def migrate(old,new):
    if old['model_kind']!=new['model_kind']:raise ValueError('Robot and field settings cannot be interchanged')
    same=old['source']['fingerprint']==new['source']['fingerprint']
    new['profile_id']=old['profile_id'];new['parameters']=copy.deepcopy(old['parameters']);new['runtime']=copy.deepcopy(old['runtime']);new['provenance']=copy.deepcopy(old['provenance']);new['adapter']=old['adapter'] if same else 'general'
    pending=[];used=set();decisions=[]
    for name,entity in new['entities'].items():
        candidate=None;l=new['source']['links'][name]
        if name in old['entities'] and old['source']['links'][name]['geometry_signature']==l['geometry_signature'] and old['source']['links'][name]['parent']==l['parent']:candidate=name
        else:
            matches=[n for n,o in old['source']['links'].items() if o['geometry_signature']==l['geometry_signature'] and o['visuals'] and n not in used]
            if len(matches)==1:candidate=matches[0]
        if candidate is not None and candidate not in used and (old['source']['links'][candidate]['joint'] or {}).get('type')==(l['joint'] or {}).get('type') and (old['source']['links'][candidate]['parent']==l['parent'] or old['source']['links'][candidate]['parent'] is not None and l['parent'] is not None and old['source']['links'][old['source']['links'][candidate]['parent']]['geometry_signature']==new['source']['links'][l['parent']]['geometry_signature']):
            used.add(candidate);cad_actuators=copy.deepcopy(entity['settings']['actuators']);entity=copy.deepcopy(old['entities'][candidate]);
            old_joint=old['source']['links'][candidate]['joint'];new_joint=l['joint']
            old_tx=[x for x in xml_source_transmissions(old) if old_joint and x['joint']==old_joint['name']]
            new_tx=[x for x in xml_source_transmissions(new) if new_joint and x['joint']==new_joint['name']]
            if old_tx!=new_tx:pending.append({'new':name,'binding_link':name,'cad_actuators':cad_actuators,'options':['Keep saved bindings','Use new CAD bindings'],'reason':'Source transmission/hardware bindings changed'})
            entity['settings']['joint']=migrate_joint(entity['settings']['joint'],old['source']['links'][candidate]['joint'],l['joint']);
            if entity['settings']['collision_strategy']=='source' and not l['collisions']:
                entity['settings']['collision_strategy']='box';entity['assumptions'].append('Source collision disappeared; retained proxy dimensions need review')
            new['entities'][name]=entity;decisions.append({'old':candidate,'new':name,'method':'unique compatible geometry/context'})
        else:
            choices=[n for n in old['entities'] if n not in used and (n==name or old['source']['links'][n]['geometry_signature']==l['geometry_signature'])]
            pending.append({'new':name,'options':['Generate defaults']+['Use settings: '+n for n in choices],'reason':'Changed or ambiguous part; select a source for retained settings'})
    mapping={d['old']:d['new'] for d in decisions}
    old_names=set(old['entities']);new_names=set(new['entities'])
    for name,entity in new['entities'].items():
        j=entity['settings']['joint']
        if j and j['parent'] in mapping:j['parent']=mapping[j['parent']]
        if j and j['parent'] not in new_names:
            pending.append({'new':name,'entity_parent':name,'options':['Use CAD parent and pose']+['Parent: '+n for n in sorted(new_names) if n!=name],'reason':'The saved joint parent was removed: '+j['parent']})
            current=new['source']['links'][name]['joint'];j['parent']=current['parent'];j['xyz']=current['xyz'];j['rpy']=current['rpy']
    old_joints={l['joint']['name'] for l in old['source']['links'].values() if l['joint']};new_joints={l['joint']['name'] for l in new['source']['links'].values() if l['joint']}
    for d in decisions:
        a=old['source']['links'][d['old']]['joint'];b=new['source']['links'][d['new']]['joint']
        if a and b:mapping[a['name']]=b['name']
    def references(value,path):
        link_arrays={'links'};joint_arrays={'omni_joints'};named={'link','joint','body','chassis','root_link','mimic'}
        if isinstance(value,dict):
            for k,v in list(value.items()):
                if path and path[-1] in ['visual_indices','collision_omissions'] and k in mapping and mapping[k]!=k:value[mapping[k]]=value.pop(k);k=mapping[k]
                if k in named and isinstance(v,str) and v in mapping:value[k]=mapping[v]
                elif k in named and isinstance(v,str) and (v in old_names-new_names or v in old_joints-new_joints):pending.append({'new':None,'runtime_path':path+[k],'options':['Map to: '+n for n in sorted(new_names if v in old_names else new_joints)]+['Disable saved model: '+str(path[0])],'reason':'Saved mechanism reference needs a new target: '+v})
                else:references(v,path+[k])
        elif isinstance(value,list):
            for i,v in enumerate(value):
                typed=path and path[-1] in link_arrays|joint_arrays
                if typed and isinstance(v,str) and v in mapping:value[i]=mapping[v]
                elif typed and isinstance(v,str) and (v in old_names-new_names or v in old_joints-new_joints):pending.append({'new':None,'runtime_path':path+[i],'options':['Map to: '+n for n in sorted(new_names if path[-1] in link_arrays else new_joints)]+['Disable saved model: '+str(path[0])],'reason':'Saved contact reference needs a new target: '+v})
                else:references(v,path+[i])
    references(new['runtime'],[])
    removed=[n for n in old['entities'] if n not in used]
    if old['adapter']=='biobuzz' and not same:pending.append({'new':None,'options':['Regenerate general field preparation'],'reason':'Authored BIOBUZZ assembly corrections cannot be safely migrated automatically. Assign roles and review all field bodies in the editor.'})
    if removed:pending.append({'new':None,'options':['Acknowledge removed parts'],'removed':removed,'reason':'Old settings must not disappear silently'})
    new['migration']={'pending':pending,'decisions':decisions,'from_revision':old['revision_id'],'old_entities':copy.deepcopy(old['entities']) if pending else {},'old_source_joints':{n:l['joint'] for n,l in old['source']['links'].items()} if pending else {}}
    if same:
        new['entities']=copy.deepcopy(old['entities']);new['migration']=copy.deepcopy(old['migration']);new['review']=copy.deepcopy(old['review'])
    else:new['review']={'state':'draft'}
    return new


def import_zip(source,library,kind,reuse=None,fresh=False):
    library=Path(library).resolve();library.mkdir(parents=True,exist_ok=True)
    folder=library/'drafts'/uid();folder.mkdir(parents=True)
    try:
        extract(source,folder/'archive');urdfs=[f for f in (folder/'archive').rglob('*') if f.suffix.lower()=='.urdf']
        if len(urdfs)!=1:raise ValueError('Expected one URDF in ZIP')
        urdf=source_copy(urdfs[0],folder/'source');p=new_profile(urdf,kind)
        if fresh and reuse:raise ValueError('Choose fresh defaults or reuse settings, not both')
        if reuse:verify_receipt(reuse)
        old=read(reuse) if reuse else None
        if old is None and not fresh:
            for file in sorted(library.glob('models/*/revisions/*/profile.json'),key=lambda f:(read(f).get('review',{}).get('state')=='ready',f.stat().st_mtime),reverse=True):
                saved=read(file)
                if saved['model_kind']==kind and saved['source']['fingerprint']==p['source']['fingerprint']:verify_receipt(file);old=saved;reuse=file;break
        if old and old['model_kind']!=kind:raise ValueError('Robot and field settings cannot be interchanged')
        if old and old['source']['fingerprint']==p['source']['fingerprint'] and old['review'].get('state')=='ready' and 'revisions' in Path(reuse).parts:
            shutil.rmtree(folder);return Path(reuse).resolve()
        if old:
            p=migrate(old,p)
            if p['adapter']=='biobuzz':shutil.copytree(Path(reuse).parent/'legacy-assets',folder/'legacy-assets')
            proof=Path(reuse).parent/'validation.json'
            if proof.is_file():shutil.copyfile(proof,folder/'validation.json')
        shutil.rmtree(folder/'archive');atomic_json(folder/'profile.json',p);compile_profile(folder/'profile.json');return folder/'profile.json'
    except Exception:
        shutil.rmtree(folder);raise


def capture(source,library,kind):
    library=Path(library).resolve();folder=library/'drafts'/uid();folder.mkdir(parents=True)
    try:
        if kind=='robot':
            config=read(Path(source)/'sim.config');urdf=source_copy((Path(source)/config['urdf']).resolve(),folder/'source',trusted=True)
            p=new_profile(urdf,'robot',{k:config[k] for k in RUNTIME_KEYS if k in config})
            for e in p['entities'].values():e['settings']['mass_mode']='source'
            p['parameters']['friction']=0.
            owners=body_owners(p['source'],p['runtime'])
            for n,e in p['entities'].items():
                if owners[n]==n and owners[n]!=p['source']['root']:e['settings']['material_override']=True;e['settings']['friction']=.5;e['settings']['restitution']=0.;e['settings']['rolling_friction']=0.;e['settings']['spinning_friction']=0.
            p['parameters']['restitution']=0.;p['parameters']['rolling_friction']=0.;p['parameters']['spinning_friction']=0.
            if p['runtime'].get('flexible_intake'):
                flex=p['runtime']['flexible_intake'];flex.setdefault('mesh_to_beam_rpy_rad',[0.,0.,math.pi/2]);indices=flex.setdefault('visual_indices',{})
                original=xml((Path(source)/config['urdf']).resolve())
                for link in original.findall('link'):
                    if link.get('name') in flex['links']:
                        matches=[i for i,v in enumerate(link.findall('visual')) if v.find('geometry/mesh') is not None and Path(v.find('geometry/mesh').get('filename')).name.lower()=='flap.stl']
                        if len(matches)==1:indices.setdefault(link.get('name'),matches[0])
            if config.get('calibration'):p['runtime']['calibration_data']=read(Path(source)/config['calibration'])
        else:
            manifest=read(source);directory=Path(source).parent;urdf=source_copy(directory/manifest['urdf'],folder/'source',trusted=True);p=new_profile(urdf,'field');p['adapter']='biobuzz'
            for m in manifest['meshes'].values():
                target=folder/'legacy-assets'/Path(m['file']).name;target.parent.mkdir(exist_ok=True);shutil.copyfile(directory/m['file'],target);m['file']=str(target.relative_to(folder));m['prepared_sha256']=sha(target)
            for inst in manifest['instances']:
                if inst['id'] in p['entities']:
                    settings=p['entities'][inst['id']]['settings'];settings['role']='piece' if inst['owner']=='piece' else 'surface' if inst['category']=='floor' else 'reference' if inst['category']=='reference' else 'decoration' if inst['category']=='decoration' else 'structure'
                    if inst['owner']=='piece':settings['collision_strategy']='sphere';settings['sphere_radius_m']=inst['radius_m'];settings['mass_mode']='override';settings['mass_kg']=inst['mass_kg'];settings['collision_xyz_m']=inst['mesh_center_m']
            p['runtime']['field_manifest']=manifest;p['parameters']['field_half_extents_m']=manifest['field_half_extents_m'];p['parameters']['collision_margin_m']=.0005;p['parameters']['floor_strategy']='generated'
        atomic_json(folder/'profile.json',p);compile_profile(folder/'profile.json');return folder/'profile.json'
    except Exception:shutil.rmtree(folder);raise


def resolve_migration(path,index,choice):
    p=read(path);item=p['migration']['pending'][index]
    if choice not in item['options']:raise ValueError('Select one of the proposed resolutions')
    if choice.startswith('Disable saved model: '):
        key=choice[21:];p['runtime'].pop(key,None)
        for dependent in {'drive':['tires'],'intake':['flexible_intake','torus_retention'],'flexible_intake':['torus_retention']}.get(key,[]):p['runtime'].pop(dependent,None)
        p['migration']['pending']=[q for q in p['migration']['pending'] if not q.get('runtime_path') or q['runtime_path'][0] in p['runtime']]
        if item not in p['migration']['pending']:p['migration']['pending'].insert(0,item)
        index=p['migration']['pending'].index(item)
    if item.get('entity_parent') and choice.startswith('Parent: '):p['entities'][item['entity_parent']]['settings']['joint']['parent']=choice[8:]
    if item.get('binding_link') and choice=='Use new CAD bindings':p['entities'][item['binding_link']]['settings']['actuators']=item['cad_actuators']
    if choice.startswith('Map to: '):
        target=p['runtime']
        for k in item['runtime_path'][:-1]:target=target[k]
        target[item['runtime_path'][-1]]=choice[8:]
    if choice.startswith('Use settings: '):
        old=choice[14:];name=item['new'];entity=copy.deepcopy(p['migration']['old_entities'][old]);entity['settings']['joint']=migrate_joint(entity['settings']['joint'],p['migration']['old_source_joints'].get(old),p['source']['links'][name]['joint']);
        if entity['settings']['joint'] and entity['settings']['joint']['parent'] not in p['entities']:entity['settings']['joint']['parent']=p['source']['links'][name]['joint']['parent']
        p['entities'][name]=entity
        # Changed geometry never silently retains a box's old dimensions.
        p['entities'][name]['settings']['collision_strategy']='visual'
        p['entities'][name]['assumptions'].append('Migrated settings; regenerated collision geometry requires review')
        for pending in p['migration']['pending']:
            if 'removed' in pending:pending['removed']=[n for n in pending['removed'] if n!=old]
    p['migration']['decisions'].append({**item,'selected':choice});p['migration']['pending'].pop(index)
    p['migration']['pending']=[q for q in p['migration']['pending'] if 'removed' not in q or q['removed']]
    p['review']={'state':'draft'};atomic_json(path,p);compile_profile(path)


def save_revision(path,library,reviewed=False):
    path=Path(path).resolve();
    if 'revisions' in path.parts:raise ValueError('Open a saved revision as an editable draft first')
    p=read(path);receipt=compile_profile(path)
    if reviewed:
        if p['migration']['pending']:raise ValueError('Resolve migration choices before review')
        proof=read(path.parent/'validation.json') if (path.parent/'validation.json').is_file() else {}
        if not proof.get('valid') or proof.get('effective_digest')!=receipt['effective_digest']:raise ValueError('Open the live preview or run validateModel for these settings before approving collision review')
        p=read(path);p['review']={'state':'ready','digest':receipt['effective_digest'],'native_validation':proof}
    else:p['review']={'state':'draft'}
    p['revision_id']=uid();destination=Path(library).resolve()/'models'/p['profile_id']/'revisions'/p['revision_id'];destination.parent.mkdir(parents=True,exist_ok=True)
    staging=destination.parent/('.saving-'+uid())
    try:
        shutil.copytree(path.parent,staging,ignore=shutil.ignore_patterns('prepared','receipt.json'));atomic_json(staging/'profile.json',p);compile_profile(staging/'profile.json');os.replace(staging,destination)
    finally:
        if staging.exists():shutil.rmtree(staging)
    return destination/'profile.json'


def export_bundle(path,destination):
    path=Path(path).resolve();verify_receipt(path)
    destination=Path(destination).resolve()
    if destination.is_relative_to(path.parent):raise ValueError('Export bundle outside the model revision')
    with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED) as out:
        for file in sorted(path.parent.rglob('*')):
            if file.is_file():out.write(file,str(file.relative_to(path.parent)))


def verify_receipt(path):
    path=Path(path);p=read(path);r=read(path.parent/'receipt.json')
    if r['profile_sha256']!=sha(path) or r['effective_digest']!=effective_digest(p):raise ValueError('Profile changed; prepare it again')
    for file,checksum in r['files'].items():
        f=(path.parent/file).resolve()
        if not f.is_relative_to(path.parent.resolve()) or sha(f)!=checksum:raise ValueError('Prepared/source asset changed: '+file)
    return r


def fork_draft(path,library):
    verify_receipt(path);folder=Path(library).resolve()/'drafts'/uid();shutil.copytree(Path(path).parent,folder)
    return folder/'profile.json'


def scene(path,robot=None,field=None):
    p={'schema_version':1,'robot_profile':os.path.relpath(Path(robot).resolve(),Path(path).resolve().parent) if robot else None,'field_profile':os.path.relpath(Path(field).resolve(),Path(path).resolve().parent) if field else None,'mode':'field-only','robot_start_xyz_m':[0.,.1,0.],'robot_start_yaw_rad':0.,'pieces':{}}
    for key in ['robot_profile','field_profile']:
        if p[key]:
            ref=(Path(path).resolve().parent/p[key]).resolve();profile=read(ref);verify_receipt(ref);p[key.replace('_profile','_identity')]={'profile_id':profile['profile_id'],'revision_id':profile['revision_id']}
    atomic_json(path,p);return Path(path)


def import_bundle(bundle,library):
    folder=Path(library).resolve()/'drafts'/uid();folder.mkdir(parents=True)
    try:extract(bundle,folder);compile_profile(folder/'profile.json');return folder/'profile.json'
    except Exception:shutil.rmtree(folder);raise


def main():
    a=argparse.ArgumentParser(description=__doc__);sub=a.add_subparsers(dest='command',required=True)
    im=sub.add_parser('import');im.add_argument('zip');im.add_argument('library');im.add_argument('--kind',choices=['robot','field'],required=True);im.add_argument('--reuse');im.add_argument('--fresh',action='store_true')
    for name in ['capture-robot','capture-field']:
        s=sub.add_parser(name);s.add_argument('source');s.add_argument('library')
    s=sub.add_parser('compile');s.add_argument('profile')
    s=sub.add_parser('units');s.add_argument('profile');s.add_argument('unit',choices=['m','mm','in'])
    s=sub.add_parser('edit');s.add_argument('profile');s.add_argument('library')
    s=sub.add_parser('duplicate');s.add_argument('profile');s.add_argument('library')
    s=sub.add_parser('scene');s.add_argument('destination');s.add_argument('--robot');s.add_argument('--field')
    s=sub.add_parser('resolve');s.add_argument('profile');s.add_argument('index',type=int);s.add_argument('choice')
    s=sub.add_parser('save');s.add_argument('profile');s.add_argument('library');s.add_argument('--reviewed',action='store_true')
    s=sub.add_parser('export');s.add_argument('profile');s.add_argument('destination')
    s=sub.add_parser('import-profile');s.add_argument('bundle');s.add_argument('library')
    args=a.parse_args()
    if args.command=='import':result=import_zip(args.zip,args.library,args.kind,args.reuse,args.fresh)
    elif args.command.startswith('capture-'):result=capture(args.source,args.library,args.command[8:])
    elif args.command=='compile':result=compile_profile(args.profile)
    elif args.command=='units':change_units(args.profile,args.unit);result=args.profile
    elif args.command=='edit':result=fork_draft(args.profile,args.library)
    elif args.command=='duplicate':
        result=fork_draft(args.profile,args.library);p=read(result);p['profile_id']=uid();p['name']+=' (alternative tuning)';atomic_json(result,p);compile_profile(result)
    elif args.command=='scene':result=scene(args.destination,args.robot,args.field)
    elif args.command=='resolve':resolve_migration(args.profile,args.index,args.choice);result=args.profile
    elif args.command=='save':result=save_revision(args.profile,args.library,args.reviewed)
    elif args.command=='export':export_bundle(args.profile,args.destination);result=args.destination
    else:result=import_bundle(args.bundle,args.library)
    print(json.dumps({'result':str(result) if isinstance(result,Path) else result},allow_nan=False))

if __name__=='__main__':main()
