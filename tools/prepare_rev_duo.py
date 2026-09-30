#!/usr/bin/env python3
"""Prepare the verified 614-link REV DUO starter-base export; preserve all original link data.
No third-party dependencies. Source and meshes remain untouched. Not a generic CAD classifier.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def rotation(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def rpy(a):
    p = math.asin(max(-1, min(1, -a[2][0])))
    if abs(math.cos(p)) > 1e-7:
        return [math.atan2(a[2][1], a[2][2]), p, math.atan2(a[1][0], a[0][0])]
    return [0, p, math.atan2(-a[0][1], a[1][1])]


def numbers(v):
    return ' '.join(f'{x:.12g}' for x in v)


def pose(j, xyz, angles=(0, 0, 0)):
    ET.SubElement(j, 'origin', xyz=numbers(xyz), rpy=numbers(angles))


def frame(robot, name):
    l = ET.SubElement(robot, 'link', name=name)
    inertial = ET.SubElement(l, 'inertial')
    ET.SubElement(inertial, 'mass', value='0')
    ET.SubElement(inertial, 'inertia', **{k: '0' for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']})
    return l


def joint(robot, name, parent, child, xyz, angles=(0, 0, 0), moving=False):
    j = ET.SubElement(robot, 'joint', name=name, type='continuous' if moving else 'fixed')
    ET.SubElement(j, 'parent', link=parent)
    ET.SubElement(j, 'child', link=child)
    pose(j, xyz, angles)
    if moving:
        ET.SubElement(j, 'axis', xyz='0 1 0')
    return j


def collision(link, xyz, kind, **params):
    c = ET.SubElement(link, 'collision')
    pose(c, xyz, (math.pi/2, 0, 0) if kind == 'cylinder' else (0, 0, 0))
    ET.SubElement(ET.SubElement(c, 'geometry'), kind, **params)


def transmission(robot, j, motor, reduction):
    t = ET.SubElement(robot, 'transmission', name=j+'_transmission')
    ET.SubElement(t, 'joint', name=j)
    a = ET.SubElement(t, 'actuator', name=motor)
    ET.SubElement(a, 'mechanicalReduction').text = str(reduction)


def prepare(source, destination):
    source = Path(source).resolve()
    destination = Path(destination).resolve()
    if destination.exists():
        raise ValueError('Destination already exists; choose a new directory to preserve existing team files')
    raw = source.read_bytes()
    if b'<!DOCTYPE' in raw.upper() or b'<!ENTITY' in raw.upper():
        raise ValueError('DTD/entities are not accepted')
    tree = ET.fromstring(raw)
    links = {l.get('name'): l for l in tree.findall('link')}
    joints = tree.findall('joint')
    if len(links) != 614 or len(joints) != 613 or any(j.get('type') != 'fixed' for j in joints):
        raise ValueError('Profile requires the verified 614-link, all-fixed REV DUO starter-base assembly')
    required = {'root', 'rev_41_1300', 'rev_41_1602', 'rev_41_1603', 'rev_41_1602_13', 'rev_41_1603_13',
                'rev_41_1190', 'rev_41_1190_29', 'tread', 'tread_1', 'rev_41_1354_default', 'rev_41_1354_default_2',
                '5mm_x_400mm_hex_shaft__rev_41_1362_', '5mm_x_400mm_hex_shaft__rev_41_1362__1'}
    if not required <= links.keys():
        raise ValueError('Export lacks required wheel, motor or gearbox components')
    identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    poses = {'root': (identity, [0, 0, 0])}
    pending = joints[:]
    while pending:
        before = len(pending)
        for j in pending[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in poses:
                continue
            a, v = poses[parent]
            o = j.find('origin')
            b = rotation(*map(float, o.get('rpy', '0 0 0').split()))
            w = mv(a, list(map(float, o.get('xyz', '0 0 0').split())))
            poses[child] = mm(a, b), [v[i]+w[i] for i in range(3)]
            pending.remove(j)
        if len(pending) == before:
            raise ValueError('Export is not a rooted tree')
    # CAD x is transverse; CAD -y points into the intake. Normalize to URDF x forward, y left.
    basis = rotation(0, 0, math.pi/2)
    poses = {name: (mm(basis, a), mv(basis, v)) for name, (a, v) in poses.items()}
    wheels = [('left_rear', 'rev_41_1190', 'leftDrive', -1), ('right_rear', 'rev_41_1190_29', 'rightDrive', 1),
              ('left_center', 'tread', 'leftDrive', -1), ('right_center', 'tread_1', 'rightDrive', 1),
              ('left_front', 'rev_41_1354_default', 'leftDrive', -1), ('right_front', 'rev_41_1354_default_2', 'rightDrive', 1)]
    for _, name, _, _ in wheels:
        v = poses[name][1]
        if abs(abs(v[1])-.187) > .01 or abs(v[2]-.0445) > .002:
            raise ValueError(f'Wheel placement differs from verified assembly: {name}')
    upper = mv(basis, [0, -.14477, .18193])
    lower = mv(basis, [0, -.19875, .088445])
    robot = ET.Element('robot', name='REV_DUO_starter_prepared')
    for material in tree.findall('material'):
        robot.append(copy.deepcopy(material))
    base = frame(robot, 'sim_base')
    groups = {}
    # Include shafts, collars and coaxial spacers; bearings and stationary mounts stay welded.
    for wheel, name, motor, reduction in wheels:
        pivot = poses[name][1]
        groups[wheel] = (pivot, {name})
        for n, (_, v) in poses.items():
            if (n.startswith('5mm_x_90mm_hex_shaft') or n.startswith('shaft_collar')) and abs(v[0]-pivot[0]) < .002 and abs(v[2]-pivot[2]) < .002 and v[1]*pivot[1] > 0:
                groups[wheel][1].add(n)
        frame(robot, wheel)
        joint(robot, wheel+'_joint', 'sim_base', wheel, pivot, moving=True)
        transmission(robot, wheel+'_joint', motor, reduction)
        # Wheel envelopes belong to the chassis's collision compound, not separate tire bodies.
        collision(base, pivot, 'cylinder', radius='.045', length='.036')
    for name, pivot in [('intake_upper', upper), ('intake_lower', lower)]:
        members = set()
        for n, (_, v) in poses.items():
            rotating = n.startswith(('flap', '5mm_x_400mm_hex_shaft', 'shaft_collar', 'rev_41_1340', '15mm_spacer', '3mm_spacer'))
            if rotating and abs(v[0]-pivot[0]) < .002 and abs(v[2]-pivot[2]) < .002:
                members.add(n)
        groups[name] = (pivot, members)
        body = frame(robot, name)
        j = joint(robot, name+'_joint', 'sim_base', name, pivot, moving=True)
        collision(body, [0, 0, 0], 'cylinder', radius='.025' if name.endswith('upper') else '.03', length='.23')
        if name.endswith('upper'):
            transmission(robot, j.get('name'), 'intake', 1)
        else:
            ET.SubElement(j, 'mimic', joint='intake_upper_joint', multiplier='1', offset='0')
    owned = {}
    for group, (_, members) in groups.items():
        for n in members:
            if n in owned:
                raise ValueError('Overlapping rotating assemblies: '+n)
            owned[n] = group
    for name, original in links.items():
        link = copy.deepcopy(original)
        for mesh in link.findall('.//mesh'):
            uri = mesh.get('filename')
            if not uri.startswith('package://'):
                raise ValueError('Profile expects native package:// mesh paths')
            package, relative = uri[len('package://'):].split('/', 1)
            if source.parent.parent.name != package:
                raise ValueError('URDF package name does not match package directory')
            path = (source.parent.parent / relative).resolve()
            if not path.is_relative_to(source.parent.parent) or not path.is_file():
                raise ValueError('Mesh outside package or absent: '+uri)
            mesh.set('filename', str(path))
        robot.append(link)
        a, v = poses[name]
        parent = owned.get(name, 'sim_base')
        pivot = groups[parent][0] if parent in groups else [0, 0, 0]
        joint(robot, 'prepared_'+name, parent, name, [v[i]-pivot[i] for i in range(3)], rpy(a))
    # Open mouth: two narrow rails and rear crossbar, rather than a box filling the intake.
    for side in [-1, 1]:
        collision(base, [.006635, side*.139, .085], 'box', size='.31 .03 .045')
    collision(base, [-.165, 0, .09], 'box', size='.03 .28 .05')
    template = Path(__file__).resolve().parents[1] / 'gui-runner/rev-duo-template'
    shutil.copytree(template, destination)
    ET.indent(robot)
    ET.ElementTree(robot).write(destination/'robot.urdf', encoding='utf-8', xml_declaration=True)
    config = json.loads((destination/'sim.config').read_text())
    config['drive']['track_width_m'] = abs(poses['tread'][1][1]-poses['tread_1'][1][1])
    (destination/'sim.config').write_text(json.dumps(config, indent=2)+'\n')
    report = {'source_sha256': hashlib.sha256(raw).hexdigest(), 'source_links': len(links),
              'preserved_visuals': len(tree.findall('.//visual')), 'cad_mass_kg': sum(float(l.find('inertial/mass').get('value')) for l in links.values() if l.find('inertial/mass') is not None),
              'drive_track_width_m': config['drive']['track_width_m'], 'assemblies': {g: sorted(m) for g, (_, m) in groups.items()},
              'limits': ['CAD mass unmeasured', 'Collision envelopes approximate', 'Aggregate planar traction, no individual tire slip',
                         'Gearbox losses uncalibrated', 'Ideal 1:1 intake chain coupling', 'Proximity capture, illustrative torus game piece']}
    (destination/'preparation-report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'assemblies'}, indent=2))
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    try:
        prepare(args.urdf, args.destination)
    except (ValueError, OSError, ET.ParseError) as error:
        parser.exit(2, f'Preparation failed: {error}\n')
