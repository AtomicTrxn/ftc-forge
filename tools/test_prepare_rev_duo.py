"""Synthetic fixture checks: source preservation, frame conversion, and rejection gates."""
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import prepare_rev_duo as prep


class PreparationTest(unittest.TestCase):
    def fixture(self, root):
        package = root/'fixture_package'
        (package/'urdf').mkdir(parents=True)
        (package/'meshes').mkdir()
        (package/'meshes'/'part.stl').write_bytes(b'fixture')
        robot = ET.Element('robot', name='source')
        required = {
            'rev_41_1300': [0, -.14477, .18193],
            'rev_41_1602': [0, .1, .0445], 'rev_41_1603': [0, .1, .0445],
            'rev_41_1602_13': [0, .1, .0445], 'rev_41_1603_13': [0, .1, .0445],
            'rev_41_1190': [.182, .137, .0445], 'rev_41_1190_29': [-.182, .137, .0445],
            'tread': [.1905, -.0066, .0445], 'tread_1': [-.1905, -.0066, .0445],
            'rev_41_1354_default': [.188, -.1506, .0445], 'rev_41_1354_default_2': [-.188, -.1506, .0445],
            '5mm_x_400mm_hex_shaft__rev_41_1362_': [.192, -.14477, .18193],
            '5mm_x_400mm_hex_shaft__rev_41_1362__1': [.192, -.19875, .088445],
            'flap': [.03, -.19875, .088445],
            **{'flap_'+str(i): [.03*i, -.19875, .088445] for i in range(1,6)},
            'polycarbonate_sheet___2mm___8mm_grid_pattern___112_x_248_mm': [0, 0, 0]}
        names = ['root']+list(required)
        names += ['part_'+str(i) for i in range(614-len(names))]
        for name in names:
            l = prep.frame(robot, name)
            l.find('inertial/mass').set('value', '.1')
            l.find('inertial/inertia').set('ixx', '.001')
            l.find('inertial/inertia').set('iyy', '.001')
            l.find('inertial/inertia').set('izz', '.001')
            visual = ET.SubElement(l, 'visual')
            ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename='package://fixture_package/meshes/part.stl')
            if name != 'root':
                prep.joint(robot, 'j_'+name, 'root', name, required.get(name, [.01, .02, .03]), (.2, math.pi/2, .3))
        source = package/'urdf'/'robot.urdf'
        ET.ElementTree(robot).write(source)
        return source, required

    def test_preserves_source_mass_visuals_and_world_poses(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, required = self.fixture(root)
            before = source.read_bytes()
            output = prep.prepare(source, root/'powered')
            result = ET.parse(output/'robot.urdf').getroot()
            self.assertEqual(before, source.read_bytes())
            self.assertEqual(614, len(result.findall('.//visual')))
            self.assertAlmostEqual(61.4, sum(float(l.find('inertial/mass').get('value')) for l in result.findall('link')))
            self.assertEqual(8, sum(j.get('type') == 'continuous' for j in result.findall('joint')))
            links = {l.get('name'): l for l in result.findall('link')}
            mounts = {j.find('child').get('link'): j for j in result.findall('joint')}
            def world(name):
                if name == 'sim_base':
                    return prep.rotation(0,0,0), [0,0,0]
                j = mounts[name]
                a, v = world(j.find('parent').get('link'))
                o = j.find('origin')
                b = prep.rotation(*map(float,o.get('rpy').split()))
                p = prep.mv(a,list(map(float,o.get('xyz').split())))
                return prep.mm(a,b), [v[i]+p[i] for i in range(3)]
            basis = prep.rotation(0,0,math.pi/2)
            expected_rotation = prep.mm(basis,prep.rotation(.2,math.pi/2,.3))
            for name, point in required.items():
                a, p = world(name)
                for actual, expected in zip(p,prep.mv(basis,point)):
                    self.assertAlmostEqual(actual,expected,places=9)
                for i in range(3):
                    for j in range(3):
                        self.assertAlmostEqual(a[i][j],expected_rotation[i][j],places=9)
            for l in links.values():
                for mesh in l.findall('.//mesh'):
                    self.assertTrue(Path(mesh.get('filename')).is_file())
            with self.assertRaises(ValueError):
                prep.prepare(source, output)

    def test_contact_mode_explicit_mass_transfer_and_ramp_proxy(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            source, _=self.fixture(root)
            before=source.read_bytes()
            output=prep.prepare(source,root/'contact',contact_models=True)
            robot=ET.parse(output/'robot.urdf').getroot()
            links={l.get('name'):l for l in robot.findall('link')}
            self.assertEqual(before,source.read_bytes())
            for name in ['flap']+['flap_'+str(i) for i in range(1,6)]:
                self.assertEqual(.005,float(links[name].find('inertial/mass').get('value')))
                self.assertAlmostEqual(.00005,float(links[name].find('inertial/inertia').get('ixx')))
            self.assertAlmostEqual(60.83,sum(float(l.find('inertial/mass').get('value')) for l in links.values()))
            ramp=links['polycarbonate_sheet___2mm___8mm_grid_pattern___112_x_248_mm']
            self.assertEqual('.248 .112 .002',ramp.find('collision/geometry/box').get('size'))
            import json
            config=json.loads((output/'sim.config').read_text())
            self.assertEqual(2,len(config['tires']['omni_joints']))
            self.assertEqual(6,len(config['flexible_intake']['links']))

    def test_rejects_different_wheel_layout_and_entities(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source, _ = self.fixture(root)
            tree = ET.parse(source)
            for j in tree.findall('joint'):
                if j.find('child').get('link') == 'tread':
                    j.find('origin').set('xyz', '.3 0 .0445')
            tree.write(source)
            with self.assertRaises(ValueError):
                prep.prepare(source,root/'bad')
            self.assertFalse((root/'bad').exists())
            source.write_text('<!DOCTYPE robot><robot/>')
            with self.assertRaises(ValueError):
                prep.prepare(source,root/'bad')


if __name__ == '__main__':
    unittest.main()
