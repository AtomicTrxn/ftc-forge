#!/usr/bin/env python3
"""Prepare the supplied BIOBUZZ Onshape assembly in meters. Standard library only.

Original CAD stays untouched. This is an explicit BIOBUZZ practice profile, not a
generic classifier or a certified match layout. Run --help for import instructions.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import shutil
import struct
import xml.etree.ElementTree as ET
import zipfile

from prepare_rev_duo import rotation, mm, mv, rpy

IDENTITY = (rotation(0, 0, 0), [0., 0., 0.])


def vector(text, length=3):
    result = list(map(float, text.split()))
    if len(result) != length or not all(math.isfinite(x) for x in result):
        raise ValueError('Invalid finite vector')
    return result


def pose(element):
    if element is None:
        return IDENTITY
    return rotation(*vector(element.get('rpy', '0 0 0'))), vector(element.get('xyz', '0 0 0'))


def combine(a, b):
    ar, at = a
    br, bt = b
    t = mv(ar, bt)
    return mm(ar, br), [at[i] + t[i] for i in range(3)]


def relative(a, b):
    ar, at = a
    inv = list(map(list, zip(*ar)))
    br, bt = b
    return mm(inv, br), mv(inv, [bt[i] - at[i] for i in range(3)])


def serial_pose(p):
    return dict(xyz_m=p[1], rpy_rad=rpy(p[0]))


def read_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f'Invalid binary STL: {path}')
    count = struct.unpack_from('<I', data, 80)[0]
    if count == 0 or count > 1_000_000 or len(data) != 84 + count * 50:
        raise ValueError(f'Invalid binary STL length/budget: {path}')
    triangles = []
    for row in struct.iter_unpack('<12fH', data[84:]):
        if not all(math.isfinite(x) for x in row[:12]):
            raise ValueError(f'Nonfinite STL: {path}')
        triangles.append(tuple(tuple(row[3 + v*3:6 + v*3]) for v in range(3)))
    return triangles


def bounds(triangles):
    return [[f(v[i] for tri in triangles for v in tri) for i in range(3)] for f in (min, max)]


def ball_dimensions(bb):
    lo, hi = bb
    return [(lo[i]+hi[i])/2 for i in range(3)], max(hi[i]-lo[i] for i in range(3))/2


def clustered(triangles, step):
    """Snap to a metric grid; every retained vertex moves <= sqrt(3)*step/2.

    This bound describes vertex displacement, not a Hausdorff/topology guarantee.
    Collision geometry and representative openings are validated separately.
    """
    seen = set()
    result = []
    for tri in triangles:
        snapped = tuple(tuple(round(x / step) for x in v) for v in tri)
        if len(set(snapped)) < 3:
            continue
        key = tuple(sorted(snapped))
        if key in seen:
            continue
        a, b, c = snapped
        u = [b[i]-a[i] for i in range(3)]
        v = [c[i]-a[i] for i in range(3)]
        if all(x == 0 for x in cross(u, v)):
            continue
        seen.add(key)
        result.append(tuple(tuple(x * step for x in v) for v in snapped))
    return result


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def write_stl(path, triangles):
    with path.open('wb') as f:
        f.write(b'FTC Forge metric visual preparation'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(triangles)))
        for tri in triangles:
            a, b, c = tri
            n = cross([b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)])
            length = math.sqrt(dot(n, n))
            n = [x/length for x in n] if length else [0, 0, 0]
            f.write(struct.pack('<12fH', *n, *(x for v in tri for x in v), 0))


def planar_hulls(triangles):
    """Merge adjacent coplanar triangles only when their union stays convex.

    Each output surface gets a 0.25 mm normal skin. Concave boundaries and holes
    remain split instead of being filled by a convex hull of the whole part.
    """
    polys, normals, edges = {}, {}, collections.defaultdict(set)
    def keys(poly):
        return [tuple(sorted((a, b))) for a, b in zip(poly, poly[1:]+poly[:1])]
    def add(pid, poly, n):
        polys[pid], normals[pid] = poly, n
        for e in keys(poly): edges[e].add(pid)
    for pid, tri in enumerate(triangles):
        poly = [tuple(round(x, 6) for x in v) for v in tri]
        a, b, c = poly
        n = cross([b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)])
        length = math.sqrt(dot(n, n))
        if length > 1e-12: add(pid, poly, [x/length for x in n])
    queue = collections.deque(e for e in edges if len(edges[e]) == 2)
    while queue:
        edge = queue.popleft()
        if len(edges[edge]) != 2: continue
        a, b = sorted(edges[edge])
        if a not in polys or b not in polys: continue
        if dot(normals[a], normals[b]) < .999999: continue
        pa, pb, n = polys[a], polys[b], normals[a]
        if any(abs(dot(n, [v[i]-pa[0][i] for i in range(3)])) > .000002 for v in pb): continue
        directed = [(x, y) for p in (pa, pb) for x, y in zip(p, p[1:]+p[:1]) if tuple(sorted((x, y))) != edge]
        successors = {x:y for x,y in directed}
        if len(successors) != len(directed): continue
        poly = [directed[0][0]]
        for _ in range(len(directed)-1): poly.append(successors[poly[-1]])
        if successors.get(poly[-1]) != poly[0] or len(set(poly)) != len(poly): continue
        if any(dot(cross([poly[(i+1)%len(poly)][k]-poly[i][k] for k in range(3)],
                         [poly[(i+2)%len(poly)][k]-poly[(i+1)%len(poly)][k] for k in range(3)]), n) < -1e-10 for i in range(len(poly))): continue
        for pid in (a, b):
            for e in keys(polys[pid]): edges[e].discard(pid)
            del polys[pid]
        # Remove straight intermediate vertices to keep native hulls small.
        reduced = []
        for i, v in enumerate(poly):
            prev, nxt = poly[i-1], poly[(i+1)%len(poly)]
            cr = cross([v[k]-prev[k] for k in range(3)], [nxt[k]-v[k] for k in range(3)])
            if dot(cr, cr) > 1e-20: reduced.append(v)
        if len(reduced) >= 3: poly = reduced
        add(a, poly, n)
        queue.extend(e for e in keys(poly) if len(edges[e]) == 2)
    return [[list(v[i]+sign*n[i]*.00025 for i in range(3)) for v in p for sign in (-1, 1)]
            for pid, p in polys.items() for n in [normals[pid]]]


def extract(source, dest):
    with zipfile.ZipFile(source) as z:
        infos = z.infolist()
        if len(infos) > 2000 or sum(i.file_size for i in infos) > 400_000_000:
            raise ValueError('Archive exceeds import budget')
        for i in infos:
            p = PurePosixPath(i.filename)
            if p.is_absolute() or '..' in p.parts or '\\' in i.filename or (i.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('Unsafe archive entry')
        for i in infos:
            if i.is_dir(): continue
            target = dest / i.filename
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(z.read(i))


def category(filename):
    if 'Soft_Tiles' in filename: return 'floor'
    if 'Scoring_Volume' in filename: return 'reference'
    if 'Pollen' in filename: return 'pollen'
    if 'Blue_Nectar' in filename: return 'blue_nectar'
    if 'Red_Nectar' in filename: return 'red_nectar'
    if 'Gaffer' in filename or 'gaffer' in filename or 'Sticker' in filename or 'April_Tag' in filename: return 'decoration'
    if 'Glass' in filename or 'Rail' in filename: return 'perimeter'
    if filename.startswith(('am_585', 'am_586', 'am_587', 'am_588', 'am_589')): return 'apparatus'
    return 'detail'


def skin_hulls(triangles):
    """Major sheet faces as thin convex patches; small edge fillets/fastener holes omitted."""
    groups = collections.defaultdict(list)
    areas = collections.defaultdict(float)
    for a, b, c in triangles:
        n = cross([b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)])
        length = math.sqrt(dot(n, n))
        if length < 1e-12: continue
        n = [x/length for x in n]
        key = tuple(round(x, 3) for x in n)+(round(dot(n, a), 4),)
        groups[key].append((a, b, c)); areas[key] += length/2
    hulls = []
    for key, tris in groups.items():
        if areas[key] < .01: continue
        n = key[:3]; normal = [x/math.sqrt(dot(n,n)) for x in n]
        axis = max(range(3), key=lambda i: abs(n[i]))
        dims = [i for i in range(3) if i != axis]
        vertices = {tuple(round(x, 7) for x in v) for tri in tris for v in tri}
        points = sorted(vertices, key=lambda p:(p[dims[0]],p[dims[1]]))
        def turn(a,b,c): return (b[dims[0]]-a[dims[0]])*(c[dims[1]]-a[dims[1]])-(b[dims[1]]-a[dims[1]])*(c[dims[0]]-a[dims[0]])
        lower, upper = [], []
        for p in points:
            while len(lower)>1 and turn(lower[-2],lower[-1],p)<=0: lower.pop()
            lower.append(p)
        for p in reversed(points):
            while len(upper)>1 and turn(upper[-2],upper[-1],p)<=0: upper.pop()
            upper.append(p)
        polygon = lower[:-1]+upper[:-1]
        poly_area = abs(sum(a[dims[0]]*b[dims[1]]-a[dims[1]]*b[dims[0]] for a,b in zip(polygon,polygon[1:]+polygon[:1])))/2/max(abs(normal[axis]),1e-9)
        if areas[key]/poly_area < .90:
            # Preserve substantial concavity, not one hull across an opening.
            hulls.extend(planar_hulls(tris))
        else:
            hulls.append([[v[i]+sign*normal[i]*.00025 for i in range(3)] for v in polygon for sign in (-1,1)])
    if not hulls or len(hulls)>1000: raise ValueError('Sheet collision budget/coverage needs a profile override')
    return hulls


def backstop_boxes(triangles, bb, step=.003):
    """Rasterize the actual XZ silhouette, then merge row runs into convex boxes.

    Keeps the L-shaped opening that a whole-part bounding box would fill.
    Conservative XY silhouette error is at most sqrt(2)*step (4.25 mm).
    """
    cells=set()
    for tri in triangles:
        points=[(v[0],v[2]) for v in tri]
        low=[min(p[i] for p in points) for i in range(2)];high=[max(p[i] for p in points) for i in range(2)]
        axes=[(1,0),(0,1)]+[(a[1]-b[1],b[0]-a[0]) for a,b in zip(points,points[1:]+points[:1])]
        for x in range(math.floor(low[0]/step),math.floor(high[0]/step)+1):
            for z in range(math.floor(low[1]/step),math.floor(high[1]/step)+1):
                if (x,z) in cells:continue
                center=((x+.5)*step,(z+.5)*step)
                overlap=True
                for axis in axes:
                    projected=[dot(p,axis) for p in points];mid=dot(center,axis);half=step/2*(abs(axis[0])+abs(axis[1]))
                    if min(projected)>mid+half+1e-12 or max(projected)<mid-half-1e-12:overlap=False;break
                if overlap:cells.add((x,z))
    rows=collections.defaultdict(list)
    for x,z in cells:rows[z].append(x)
    boxes=[];active={}
    for z in range(min(rows),max(rows)+2):
        xs=sorted(rows[z]);runs=[]
        for x in xs:
            if runs and x==runs[-1][1]+1:runs[-1][1]=x
            else:runs.append([x,x])
        current=set(map(tuple,runs))
        for run,start in list(active.items()):
            if run not in current:
                x0,x1=run;z0=start;z1=z
                boxes.append(dict(center_m=[(x0+x1+1)*step/2,(bb[0][1]+bb[1][1])/2,(z0+z1)*step/2],
                    size_m=[(x1-x0+1)*step,bb[1][1]-bb[0][1],(z1-z0)*step]))
                del active[run]
        for run in current:active.setdefault(run,z)
    return boxes


def collision(filename, triangles, bb):
    lo, hi = bb
    if 'Flower_Backstop' in filename:
        return dict(kind='boxes',boxes=backstop_boxes(triangles,bb),approximation='Actual silhouette rasterized at 3 mm; thickness preserved')
    if any(s in filename for s in ['Glass', 'Rail', 'A_Frame_Leg', 'A_Frame_Top_Bar', 'Basket_Base_Tube', 'Churro', 'HIPS_Pipe', 'Peanut_Support', 'ACM_Panel', 'Sheet_Metal_Foot_Bar']):
        return dict(kind='box', center_m=[(lo[i]+hi[i])/2 for i in range(3)], size_m=[hi[i]-lo[i] for i in range(3)])
    if 'Flower_Layer' in filename:
        # Explicit practice annulus, 32 segments; preserves the 4-inch upper opening.
        # Lower openings differ; source envelope and native ball-clearance gates are retained.
        inner = .0508 if 'Layer_C' in filename else .046 if 'Layer_B' in filename else .0355
        return dict(kind='ring', inner_radius_m=inner, outer_radius_m=.061,
                    bottom_m=lo[2], top_m=hi[2], segments=32,
                    approximation='Annulus proxy; tabs and molded detail omitted')
    if any(s in filename for s in ['Hive_Goal_Top_Skin', 'Hive_Goal_Back_Skin', 'Hive_Goal_Bottom_Skin']):
        return dict(kind='hulls', hulls_m=skin_hulls(triangles), approximation='Major sheet faces; small fastener holes/edge fillets omitted; 0.25 mm skin')
    # Load-bearing foot/corner and brackets use small offline convex surface patches.
    # Ribs are covered by the adjacent cell skins; do not fill the cell with their full hull.
    return None


def prepare(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists(): raise ValueError('Destination exists; select a new output directory')
    if source.suffix.lower()=='.urdf' and destination.is_relative_to(source.parent.parent): raise ValueError('Output must be outside the source package')
    destination.mkdir(parents=True)
    try:
        raw_dir = destination / 'source'
        if source.suffix.lower() == '.zip':
            extract(source, raw_dir)
            paths = list(raw_dir.rglob('*.urdf'))
            if len(paths) != 1: raise ValueError('Expected one field URDF')
            urdf = paths[0]
            source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        else:
            if source.suffix.lower() != '.urdf': raise ValueError('Source must be ZIP or URDF')
            package = source.parent.parent.resolve()
            if any(p.is_symlink() for p in package.rglob('*')): raise ValueError('Source package contains symlinks')
            shutil.copytree(package, raw_dir/package.name)
            urdf = raw_dir/package.name/source.relative_to(package)
            source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        data = urdf.read_bytes()
        if b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper(): raise ValueError('DTD/entities not accepted')
        robot = ET.fromstring(data)
        if robot.get('name') != 'am_5850_biobuzz': raise ValueError('This profile requires am_5850_biobuzz')
        links = {l.get('name'):l for l in robot.findall('link')}
        if len(links) != len(robot.findall('link')): raise ValueError('Duplicate link')
        joints = robot.findall('joint')
        parents = {j.find('child').get('link'):j for j in joints}
        if len(parents) != len(joints) or set(links)-set(parents) != {'root'}: raise ValueError('Expected one rooted tree')
        children = collections.defaultdict(list)
        for j in joints: children[j.find('parent').get('link')].append(j)
        poses, owners = {}, {}
        hives = {}
        def visit(name, p, owner):
            if name in poses: raise ValueError('Cycle in URDF')
            poses[name], owners[name] = p, owner
            for j in children[name]:
                child = j.find('child').get('link')
                jp = combine(p, pose(j.find('origin')))
                next_owner = owner
                if j.get('type') == 'revolute':
                    next_owner = j.get('name')
                    hives[next_owner] = dict(id=next_owner, **serial_pose(jp), axis=vector(j.find('axis').get('xyz')),
                        lower_rad=float(j.find('limit').get('lower')), upper_rad=float(j.find('limit').get('upper')),
                        initial_rad=0., damping=.08, mass_kg=0., members=[])
                    hives[next_owner]['spring_peak_torque_nm']=1.5
                visit(child, jp, next_owner)
        visit('root', IDENTITY, 'fixed')
        if set(poses) != set(links): raise ValueError('Unresolved field links')
        # Onshape fastened groups containing the four CELLS hang directly from root,
        # disconnected from the revolute pivot groups. Reassign whole groups after
        # baking poses, using the colored rib identity and matching pivot X side.
        cell_groups = []
        def branch(name):
            yield name
            for j in children[name]: yield from branch(j.find('child').get('link'))
        for j in children['root']:
            names = list(branch(j.find('child').get('link')))
            ribs = [m.get('filename') for n in names for m in links[n].findall('.//mesh') if 'Goal_Rib' in m.get('filename','')]
            if not ribs: continue
            colors = {'blue' if 'blue' in rib.lower() else 'red' for rib in ribs}
            if len(colors)!=1 or len(ribs)!=2: raise ValueError('Unexpected disconnected CELL group')
            color = colors.pop()
            hive = next(h for h in hives.values() if (h['xyz_m'][0]>0)==(color=='blue'))
            for name in names: owners[name] = hive['id']
            cell_groups.append(dict(root=j.find('child').get('link'), color=color, owner=hive['id'], members=names))
        if len(cell_groups)!=4: raise ValueError('Expected four disconnected CELL groups')
        mesh_dir = destination/'meshes'
        mesh_dir.mkdir()
        cache, instances, piece_counts = {}, [], collections.Counter()
        for name, link in links.items():
            for v in link.findall('visual'):
                m = v.find('geometry/mesh')
                if m is None: raise ValueError('Expected STL visual')
                ref = m.get('filename', '')
                if not ref.startswith('package://am_5850_biobuzz/'): raise ValueError('Unexpected package reference')
                rel = PurePosixPath(ref.removeprefix('package://am_5850_biobuzz/'))
                if rel.is_absolute() or '..' in rel.parts: raise ValueError('Mesh escapes package')
                path = urdf.parent.parent/str(rel)
                scale = vector(m.get('scale', '1 1 1'))
                if scale != [1., 1., 1.]: raise ValueError('Profile expects meter meshes at scale 1')
                fn = path.name
                if fn not in cache:
                    tris = read_stl(path)
                    bb = bounds(tris)
                    cat = category(fn)
                    # Small hardware is optional full detail, not part of the standard view.
                    reduced = clustered(tris, .0005) if len(tris) > 10_000 else tris
                    if len(reduced) > 100_000: raise ValueError(f'Visual budget exceeded after preparation: {fn}')
                    write_stl(mesh_dir/fn, reduced)
                    cache[fn] = dict(file='meshes/'+fn, category=cat, source_triangles=len(tris), triangles=len(reduced),
                        bounds_m=bb, prepared_bounds_m=bounds(reduced), max_vertex_displacement_m=math.sqrt(3)*.00025 if len(tris)>10_000 else 0,
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest(), collision=collision(fn, tris, bb))
                meta = cache[fn]
                cat = meta['category']
                vp = combine(poses[name], pose(v.find('origin')))
                color = v.find('material/color')
                inst = dict(id=name, mesh=fn, category=cat, owner=owners[name], **serial_pose(vp),
                    rgba=vector(color.get('rgba'), 4) if color is not None else [.65, .65, .65, 1.])
                inertial = link.find('inertial')
                inst['mass_kg'] = float(inertial.find('mass').get('value')) if inertial is not None else 0.
                if cat in ('pollen', 'blue_nectar', 'red_nectar'):
                    piece_counts[cat] += 1
                    inst['owner'] = 'piece'
                    center, radius = ball_dimensions(meta['bounds_m'])
                    inst['radius_m'] = radius
                    inst['mesh_center_m'] = center
                    inst['source_pose'] = serial_pose(vp)
                elif owners[name] != 'fixed':
                    hives[owners[name]]['members'].append(name)
                    hives[owners[name]]['mass_kg'] += inst['mass_kg']
                instances.append(inst)
        if piece_counts != {'pollen':40, 'blue_nectar':8, 'red_nectar':8}: raise ValueError('Unexpected scoring element inventory')
        if len(hives) != 2: raise ValueError('Expected two HIVE pivots')
        for hive in hives.values():
            samples = []
            for name, l in links.items():
                if owners[name] != hive['id']: continue
                if any(category(m.get('filename').split('/')[-1]) in ('pollen','blue_nectar','red_nectar','reference') for m in l.findall('.//mesh')): continue
                inertial = l.find('inertial')
                if inertial is None: continue
                mass = float(inertial.find('mass').get('value'))
                if mass <= 0: continue
                ip = combine(poses[name], pose(inertial.find('origin')))
                it = inertial.find('inertia')
                tensor = [[float(it.get('i'+a+b)) if it.get('i'+a+b) is not None else float(it.get('i'+b+a)) for b in 'xyz'] for a in 'xyz']
                samples.append(dict(mass_kg=mass, **serial_pose(ip), inertia_kg_m2=tensor))
            hive['inertials'] = samples
            hive['mass_kg'] = sum(s['mass_kg'] for s in samples)
            hive['com_xyz_m'] = [sum(s['mass_kg']*s['xyz_m'][i] for s in samples)/hive['mass_kg'] for i in range(3)]
        # Source glass interior planes: select the planes whose broad face is near +/-X/Y.
        planes = [[], []]
        for i in instances:
            if 'Glass' not in i['mesh']: continue
            lo, hi = cache[i['mesh']]['bounds_m']
            p = (rotation(*i['rpy_rad']), i['xyz_m'])
            points = [combine(p, (IDENTITY[0], [x,y,z]))[1] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])]
            for axis in range(2):
                low, high = min(v[axis] for v in points), max(v[axis] for v in points)
                if high-low < .01 and abs((low+high)/2) > 1.7: planes[axis].append(low if low>0 else high)
        half = [sum(abs(x) for x in values)/len(values) for values in planes]
        if any(not 1.7<x<1.9 for x in half): raise ValueError('Invalid interior planes')
        # A reproducible practice inventory on the floor, clear of frame and FLOWERS.
        # CAD staging/loaded-cell poses remain in source_pose and a source preview layout.
        pieces = [i for i in instances if i['owner']=='piece']
        for k, inst in enumerate(pieces):
            side = -1 if k < 28 else 1
            col, row = divmod(k % 28, 7)
            inst['xyz_m'] = [side*(.82+col*.22), -.72+row*.24, inst['radius_m']+.002]
            inst['rpy_rad'] = [0.,0.,0.]
        manifest = dict(version=1, name='BIOBUZZ CAD practice field', units='m', source_sha256=source_hash,
            urdf=str(urdf.relative_to(destination)), field_half_extents_m=half, floor_top_m=0.,
            layout='practice', nominal_field_width_m=3.6576, piece_counts=dict(piece_counts),
            cell_group_corrections=cell_groups,
            approximations=['Source interior differs from nominal 144-inch field; CAD dimensions preserved.',
                'Practice ball layout replaces source staged/penetrating poses; not a verified match setup.',
                'Four disconnected CAD CELL groups explicitly assigned to their matching HIVE pivot.',
                'Ball collision uses solid spheres; shell holes and rubber compliance are not resolved.',
                'HIVE uses an uncalibrated passive 1.5 Nm two-position spring, damping 0.08 and CAD mass/inertia.',
                'Flower collision uses annuli; molded tabs/fillets omitted.',
                'Cell collision follows skins; molded rib detail is visual only.'],
            meshes=cache, instances=instances, hives=list(hives.values()))
        (destination/'field.json').write_text(json.dumps(manifest, indent=2)+'\n')
        summary = dict(source_sha256=source_hash, source_units='m', scale=1, field_interior_m=[2*x for x in half],
            nominal_interior_m=3.6576, pieces=dict(piece_counts), hives=len(hives),
            visual_triangles=sum(m['triangles'] for m in cache.values()),
            standard_instance_triangles=sum(cache[i['mesh']]['triangles'] for i in instances if i['category'] not in ('detail','reference')),
            surface_hulls={k:len(v['collision']['hulls_m']) for k,v in cache.items() if v['collision'] and v['collision']['kind']=='hulls'},
            approximations=manifest['approximations'])
        (destination/'AUDIT.json').write_text(json.dumps(summary, indent=2)+'\n')
        return summary
    except Exception:
        shutil.rmtree(destination)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='BIOBUZZ ZIP or original field URDF')
    parser.add_argument('destination', type=Path, help='New prepared field directory')
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.destination), indent=2))
