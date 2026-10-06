"""Preparation gates without redistributing the supplied CAD."""
import math
from pathlib import Path
import tempfile
import unittest
import zipfile
from prepare_biobuzz import extract, read_stl, write_stl, clustered, bounds, planar_hulls, backstop_boxes, ball_dimensions, category, combine, relative, IDENTITY
from prepare_rev_duo import rotation


class FieldPreparationTests(unittest.TestCase):
    def test_rejects_traversal_absolute_backslash_and_symlink_entries(self):
        for name in ['../escape', '/absolute', 'foo\\bar', 'safe']:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                archive = Path(tmp)/'x.zip'
                with zipfile.ZipFile(archive, 'w') as z:
                    i = zipfile.ZipInfo(name)
                    if name == 'safe': i.external_attr = 0o120777 << 16
                    z.writestr(i, 'untrusted')
                with self.assertRaises(ValueError): extract(archive, Path(tmp)/'out')
                self.assertFalse((Path(tmp)/'out').exists())

    def test_expanded_byte_budget_is_checked_before_writing_and_streamed_bytes_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive = Path(tmp)/'larger.zip'
            content = b'robot geometry' * 100000
            with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as z:
                z.writestr('package/mesh.stl', content)
            with self.assertRaisesRegex(ValueError, 'import budget'):
                extract(archive, Path(tmp)/'too-small', len(content)-1)
            self.assertFalse((Path(tmp)/'too-small').exists())
            extract(archive, Path(tmp)/'accepted', len(content))
            self.assertEqual(content, (Path(tmp)/'accepted/package/mesh.stl').read_bytes())
            for budget in [0, -1, True, 3*1024*1024*1024]:
                with self.assertRaises(ValueError): extract(archive, Path(tmp)/'invalid', budget)

    def test_metric_clustering_is_deterministic_and_dimension_error_bounded(self):
        triangles = [((.00013, .00012, 0), (.10023, 0, 0), (0, .20021, 0))]
        reduced = clustered(triangles, .0005)
        self.assertEqual(reduced, clustered(triangles, .0005))
        for a,b in zip(triangles[0], reduced[0]):
            self.assertLessEqual(math.dist(a,b), math.sqrt(3)*.00025)
        for row, ref in zip(bounds(reduced), bounds(triangles)):
            for a,b in zip(row,ref): self.assertLessEqual(abs(a-b),.00025)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'test.stl';write_stl(path,reduced)
            self.assertEqual(len(read_stl(path)),1)
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaises(ValueError):read_stl(path)

    def test_bakes_world_pose_before_detaching_nested_parts(self):
        parent=(rotation(0,0,math.pi/2),[1,2,3])
        child=combine(parent,(IDENTITY[0],[.5,0,0]))
        self.assertAlmostEqual(child[1][0],1)
        self.assertAlmostEqual(child[1][1],2.5)
        restored=relative(parent,child)
        self.assertAlmostEqual(restored[1][0],.5)
        self.assertAlmostEqual(restored[1][1],0)

    def test_convex_merging_preserves_a_concave_opening(self):
        # L-shaped surface, three unit squares, no triangle over the fourth square.
        triangles=[]
        for x,y in [(0,0),(1,0),(0,1)]:
            a,b,c,d=(x,y,0),(x+1,y,0),(x+1,y+1,0),(x,y+1,0)
            triangles.extend([(a,b,c),(a,c,d)])
        hulls=planar_hulls(triangles)
        self.assertGreater(len(hulls),1)
        for hull in hulls:
            self.assertFalse(any(p[0]>1 and p[1]>1 for p in hull))

    def test_explicit_piece_reference_floor_classification(self):
        self.assertEqual(category('am_5851__Pollen.stl'),'pollen')
        self.assertEqual(category('am_5852__Blue_Nectar.stl'),'blue_nectar')
        self.assertEqual(category('am_5852__Red_Nectar.stl'),'red_nectar')
        self.assertEqual(category('Flower_Scoring_Volume.stl'),'reference')
        self.assertEqual(category('am_2499_Center__FIRST_Tech_Challenge_Field_Soft_Tiles.stl'),'floor')

    def test_backstop_silhouette_keeps_large_opening_clear(self):
        triangles=[]
        for x,z in [(0,0),(.02,0),(0,.02)]:
            a,b,c,d=(x,0,z),(x+.02,0,z),(x+.02,0,z+.02),(x,0,z+.02)
            triangles.extend([(a,b,c),(a,c,d)])
        boxes=backstop_boxes(triangles,[[0,-.003,0],[.04,.003,.04]])
        self.assertGreater(len(boxes),1)
        for box in boxes:
            center=box['center_m'];size=box['size_m']
            self.assertFalse(abs(.032-center[0])<size[0]/2 and abs(.032-center[2])<size[2]/2)
            self.assertAlmostEqual(size[1],.006)

    def test_offset_mesh_origin_does_not_inflate_ball_radius(self):
        bb=[[-.046,-.046,-.0578],[.046,.046,.0328]]
        center,radius=ball_dimensions(bb)
        self.assertAlmostEqual(radius,.046)
        self.assertAlmostEqual(center[2],-.0125)


if __name__=='__main__': unittest.main()
