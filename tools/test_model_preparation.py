import copy
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
import model_preparation as m

URDF='''<robot name="other"><link name="base"><visual><geometry><box size="1 1 .02"/></geometry></visual></link><link name="ball"><visual><geometry><sphere radius=".04"/></geometry></visual></link><joint name="ball_mount" type="fixed"><parent link="base"/><child link="ball"/><origin xyz="0 0 .06"/></joint></robot>'''
class ModelPreparationTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.library=self.root/'library';self.package=self.root/'cad.zip';self.zip(URDF)
    def tearDown(self):self.tmp.cleanup()
    def zip(self,xml):
        with zipfile.ZipFile(self.package,'w') as z:z.writestr('other/urdf/model.urdf',xml)
    def load(self,kind='robot',reuse=None):return m.import_zip(self.package,self.library,kind,reuse)
    def ready(self,path):
        r=m.compile_profile(path);m.atomic_json(path.parent/'validation.json',{'valid':True,'effective_digest':r['effective_digest']});return m.save_revision(path,self.library,True)
    def test_default_generation_and_source_preservation(self):
        original=self.package.read_bytes();p=self.load();data=m.read(p);self.assertEqual('visual',data['entities']['base']['settings']['collision_strategy']);self.assertEqual('draft',data['review']['state']);self.assertEqual(original,self.package.read_bytes());self.assertEqual('1.0 1.0 0.02',m.xml(p.parent/'prepared/robot.urdf').find('.//collision/geometry/box').get('size'))
    def test_review_exact_revision_portability_and_old_revision(self):
        p=self.load();saved=self.ready(p);r=m.verify_receipt(saved);self.assertTrue(r['ready']);bundle=self.root/'profile.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'elsewhere');self.assertEqual(m.effective_digest(m.read(saved)),m.effective_digest(m.read(restored)));draft=m.fork_draft(saved,self.library);data=m.read(draft);data['parameters']['friction']=.9;m.atomic_json(draft,data);m.compile_profile(draft);self.assertFalse(m.verify_receipt(draft)['ready']);self.assertTrue(m.verify_receipt(saved)['ready']);self.assertRaises(ValueError,m.save_revision,draft,self.library,True)
    def test_identical_repacked_cad_reuses_settings(self):
        p=self.load();data=m.read(p);data['parameters']['friction']=.91;data['runtime']['calibration_data']={'battery':{'v_internal':12,'r_battery':.03}};m.atomic_json(p,data);saved=self.ready(p);self.zip(URDF);q=self.load();self.assertEqual(.91,m.read(q)['parameters']['friction']);self.assertTrue(m.verify_receipt(q)['ready']);self.assertEqual(m.read(saved)['review'],m.read(q)['review']);m.save_revision(m.fork_draft(saved,self.library),self.library);self.assertEqual(saved,self.load());self.assertRaises(ValueError,self.load,'field',saved)
    def test_fresh_import_can_recover_without_a_damaged_saved_profile(self):
        saved=self.ready(self.load());(saved.parent/'prepared/robot.urdf').write_text('damaged');fresh=m.import_zip(self.package,self.library,'robot',fresh=True);self.assertFalse(m.verify_receipt(fresh)['ready']);self.assertNotEqual(m.read(saved)['profile_id'],m.read(fresh)['profile_id'])
    def test_changed_geometry_requires_choices_and_review(self):
        saved=self.ready(self.load());self.zip(URDF.replace('1 1 .02','2 1 .02'));q=self.load(reuse=saved);data=m.read(q);self.assertFalse(m.verify_receipt(q)['ready']);self.assertTrue(data['migration']['pending']);m.resolve_migration(q,0,'Generate defaults');self.assertTrue(m.read(q)['migration']['decisions']);self.assertTrue(m.verify_receipt(saved)['ready'])
    def test_renamed_unique_part_migrates_contact_references(self):
        p=self.load();data=m.read(p);data['runtime']['contacts']={'links':['ball']};m.atomic_json(p,data);saved=self.ready(p);self.zip(URDF.replace('ball','sphere'));q=self.load(reuse=saved);self.assertEqual(['sphere'],m.read(q)['runtime']['contacts']['links']);self.assertEqual(m.read(saved)['entities']['ball']['id'],m.read(q)['entities']['sphere']['id']);self.assertFalse(m.verify_receipt(q)['ready'])
    def test_link_rename_does_not_rebind_a_same_named_motor(self):
        p=self.load();data=m.read(p);data['runtime']['intake']={'motor':'ball','links':['ball']};m.atomic_json(p,data);saved=self.ready(p);self.zip(URDF.replace('ball','sphere'));q=self.load(reuse=saved);self.assertEqual('ball',m.read(q)['runtime']['intake']['motor']);self.assertEqual(['sphere'],m.read(q)['runtime']['intake']['links'])
    def test_source_transmission_change_offers_binding_choice(self):
        tx='<transmission name="t"><joint name="ball_mount"/><actuator name="old_motor"><mechanicalReduction>-2</mechanicalReduction></actuator></transmission>'
        self.zip(URDF.replace('</robot>',tx+'</robot>'));saved=self.ready(self.load());self.zip(URDF.replace('</robot>',tx.replace('old_motor','new_motor')+'</robot>'));q=self.load(reuse=saved);items=m.read(q)['migration']['pending'];i=next(i for i,x in enumerate(items) if x.get('binding_link'));m.resolve_migration(q,i,'Use new CAD bindings');self.assertEqual('new_motor',m.read(q)['entities']['ball']['settings']['actuators'][0]['name']);self.assertEqual(-2,m.read(q)['entities']['ball']['settings']['actuators'][0]['mechanicalReduction'])
    def test_nested_scene_directory_and_material_body_ownership(self):
        saved=self.ready(self.load());m.scene(self.root/'nested/scenes/a.json',saved);self.assertTrue((self.root/'nested/scenes/a.json').is_file());p=m.fork_draft(saved,self.library);data=m.read(p);data['entities']['ball']['settings']['material_override']=True;m.atomic_json(p,data);self.assertRaisesRegex(ValueError,'per rigid body',m.compile_profile,p)
    def test_removed_contact_reference_can_disable_dependent_model(self):
        p=self.load();data=m.read(p);data['runtime']['flexible_intake']={'links':['ball']};data['runtime']['torus_retention']={'measured':123};m.atomic_json(p,data);saved=self.ready(p);self.zip('<robot name="other"><link name="base"><visual><geometry><box size="1 1 .02"/></geometry></visual></link></robot>');q=self.load(reuse=saved);items=m.read(q)['migration']['pending'];i=next(i for i,x in enumerate(items) if x.get('runtime_path'));m.resolve_migration(q,i,'Disable saved model: flexible_intake');self.assertNotIn('flexible_intake',m.read(q)['runtime']);self.assertNotIn('torus_retention',m.read(q)['runtime'])
    def test_calibrated_joint_overrides_survive_source_pose_changes(self):
        p=self.load();data=m.read(p);joint=data['entities']['ball']['settings']['joint'];joint.update(type='continuous',axis=[0,1,0]);m.atomic_json(p,data);saved=self.ready(p);self.zip(URDF.replace('0 0 .06','0 0 .09'));q=self.load(reuse=saved);joint=m.read(q)['entities']['ball']['settings']['joint'];self.assertEqual('continuous',joint['type']);self.assertEqual([0,1,0],joint['axis']);self.assertEqual([0,0,.09],joint['xyz'])
    def test_edited_source_or_artifact_is_rejected(self):
        p=self.load();f=p.parent/'prepared/robot.urdf';f.write_text(f.read_text()+' ');self.assertRaises(ValueError,m.verify_receipt,p);m.compile_profile(p);f=p.parent/'source/model.urdf';f.write_text(URDF.replace('1 1 .02','3 1 .02'));self.assertRaises(ValueError,m.compile_profile,p)
    def test_field_piece_split_keeps_pose_and_settings(self):
        p=self.load('field');data=m.read(p);data['entities']['ball']['settings']['role']='piece';m.atomic_json(p,data);r=m.compile_profile(p);self.assertEqual(1,len(r['artifacts']['pieces']));self.assertEqual([0,0,.06],r['artifacts']['pieces'][0]['xyz_m']);self.assertEqual(1,len(m.xml(p.parent/'prepared/field.urdf').findall('link')))
    def test_units_convert_generated_boxes_only(self):
        # Primitive visuals remain source, while their generated inertia dimensions become metric.
        p=self.load();m.change_units(p,'mm');self.assertEqual([.001,.001,.00002],m.read(p)['entities']['base']['settings']['box_size_m']);self.assertEqual('0.001 0.001 2e-05',m.xml(p.parent/'prepared/robot.urdf').find('.//collision/geometry/box').get('size'))
    def test_invalid_joint_and_inertia_fail_before_save(self):
        p=self.load();data=m.read(p);data['entities']['ball']['settings']['joint']['type']='revolute';m.atomic_json(p,data);self.assertRaises(ValueError,m.compile_profile,p)
    def test_archive_path_escape_and_schema_rejected(self):
        with zipfile.ZipFile(self.package,'w') as z:z.writestr('../escape.urdf',URDF)
        self.assertRaises(ValueError,self.load)
        self.zip(URDF);p=self.load();data=m.read(p);data['schema_version']=999;m.atomic_json(p,data);self.assertRaises(ValueError,m.compile_profile,p)
    def test_scene_does_not_edit_model_and_saved_revision_is_immutable(self):
        saved=self.ready(self.load());before=saved.read_bytes();scene=self.root/'scene.json';m.scene(scene,saved);self.assertEqual(before,saved.read_bytes());self.assertEqual(m.read(saved)['revision_id'],m.read(scene)['robot_identity']['revision_id']);self.assertRaises(ValueError,m.save_revision,saved,self.library)
if __name__=='__main__':unittest.main()
