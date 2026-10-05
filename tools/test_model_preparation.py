import math
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
    def test_sensors_and_field_behavior_survive_bundle_and_changed_cad_review(self):
        p=self.load();data=m.read(p)
        data['runtime']['sensors']=[{'name':'range','type':'distance','link':'base','xyz_m':[.1,0,.1],'update_hz':30,'latency_ms':50,'noise_std_m':.002,'seed':7}]
        data['runtime']['field_behavior']={'schema_version':1,'clock':{'auto_s':30,'transition_s':8,'teleop_s':120},'rules':[{'id':'target','alliance':'neutral','mode':'entry_once','types':['pollen'],'min_xyz_m':[.5,-.2,0],'max_xyz_m':[1,.2,.5],'points':2}], 'tags':[{'id':7,'name':'target','size_m':.16,'xyz_m':[1,0,.4],'rpy_rad':[0,0,math.pi]}]}
        m.atomic_json(p,data);saved=self.ready(p);bundle=self.root/'sensors.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable');self.assertEqual(data['runtime'],m.read(restored)['runtime'])
        self.zip(URDF.replace('0 0 .06','0 0 .07'));changed=m.import_zip(self.package,self.root/'portable','robot',restored);self.assertEqual(data['runtime'],m.read(changed)['runtime']);self.assertFalse(m.verify_receipt(changed)['ready'])
        self.zip(URDF.replace('base','renamedbase'));renamed=m.import_zip(self.package,self.root/'portable','robot',changed);self.assertEqual('renamedbase',m.read(renamed)['runtime']['sensors'][0]['link']);self.assertEqual(data['runtime']['field_behavior'],m.read(renamed)['runtime']['field_behavior']);self.assertFalse(m.verify_receipt(renamed)['ready'])
        for change in ['sensor','rule','tag']:
            bad=copy.deepcopy(data)
            if change=='sensor':bad['runtime']['sensors'][0]['update_hz']=999
            if change=='rule':bad['runtime']['field_behavior']['rules'][0]['points']=.5
            if change=='tag':bad['runtime']['field_behavior']['tags']*=2
            with self.assertRaises(ValueError):m.validate(bad)

    def test_rotating_wheels_preserve_settings_across_export_and_cad_migration(self):
        saved,xml=self.measured_robot();p=m.fork_draft(saved,self.library);data=m.read(p)
        data['runtime']['rotating_wheels']={'reflected_motor_inertia_kg_m2':.002}
        data['runtime']['drive_contacts']['enabled']=False;data['parameters']['chassis_lock_level']=False
        m.atomic_json(p,data);saved=self.ready(p);bundle=self.root/'rotating.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable')
        self.assertEqual(data['runtime'],m.read(restored)['runtime']);self.assertFalse(m.read(restored)['parameters']['chassis_lock_level'])
        self.zip(xml.replace('wheel_joint','renamed_joint'));changed=m.import_zip(self.package,self.root/'portable','robot',restored)
        self.assertEqual(data['runtime'],m.read(changed)['runtime']);self.assertFalse(m.verify_receipt(changed)['ready'])

    def test_wheel_settings_round_trip_and_migration_preserve_reviewed_contact_mode(self):
        p=self.load();data=m.read(p);cfg=data['runtime']['drive_contacts'];self.assertTrue(cfg['enabled'])
        cfg.update(min_support_normal_y=.83,max_contact_gap_m=.001,rolling_resistance_coefficient=.02);m.atomic_json(p,data);saved=self.ready(p);before=saved.read_bytes()
        bundle=self.root/'contacts.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable');self.assertEqual(cfg,m.read(restored)['runtime']['drive_contacts'])
        self.zip(URDF.replace('0 0 .06','0 0 .07'));migrated=m.import_zip(self.package,self.root/'portable','robot',restored);self.assertEqual(cfg,m.read(migrated)['runtime']['drive_contacts']);self.assertFalse(m.verify_receipt(migrated)['ready']);self.assertEqual(before,saved.read_bytes())
        edited=m.read(migrated);edited['runtime']['drive_contacts']['max_contact_gap_m']=.1;m.atomic_json(migrated,edited);self.assertRaises(ValueError,m.compile_profile,migrated)
    def test_independent_wheel_grip_bundle_reuse_and_renamed_cad_preserve_tuning(self):
        saved,xml=self.measured_robot();p=m.fork_draft(saved,self.library);data=m.read(p)
        data['runtime']['drive_contacts']['wheel_friction']=[{'joint':'wheel_joint','friction':.91}]
        data['provenance']['runtime/drive_contacts/wheel_friction/0/friction']='measured manually: tread/field sliding test'
        m.atomic_json(p,data);saved=self.ready(p);before=saved.read_bytes();bundle=self.root/'grip.zip'
        m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable');self.assertEqual(data['runtime'],m.read(restored)['runtime'])
        self.assertEqual(data['provenance'],m.read(restored)['provenance']);self.assertTrue(m.verify_receipt(restored)['ready'])
        self.assertEqual(saved,self.load());self.zip(xml.replace('wheel~left','renamed_wheel').replace('wheel_joint','renamed_joint'))
        changed=m.import_zip(self.package,self.root/'portable','robot',restored);profile=m.read(changed)
        self.assertEqual([{'joint':'renamed_joint','friction':.91}],profile['runtime']['drive_contacts']['wheel_friction'])
        self.assertEqual('measured manually: tread/field sliding test',profile['provenance']['runtime/drive_contacts/wheel_friction/0/friction'])
        self.assertFalse(m.verify_receipt(changed)['ready']);self.assertEqual(before,saved.read_bytes())
    def test_missing_grip_joint_can_select_a_replacement_without_disabling_native_support(self):
        saved,xml=self.measured_robot();p=m.fork_draft(saved,self.library);data=m.read(p);data['runtime']['drive_contacts']['wheel_friction']=[{'joint':'wheel_joint','friction':.81}];m.atomic_json(p,data);saved=self.ready(p)
        self.zip(xml.replace('wheel_joint','replacement_joint').replace('radius=".04"','radius=".06"'));q=self.load(reuse=saved)
        pending=m.read(q)['migration']['pending'];i=next(i for i,item in enumerate(pending) if item.get('runtime_path'))
        self.assertIn('Map to: replacement_joint',pending[i]['options']);self.assertIn('Remove saved wheel grip override',pending[i]['options']);self.assertFalse(any('Disable saved model' in c for c in pending[i]['options']))
        m.resolve_migration(q,i,'Map to: replacement_joint');cfg=m.read(q)['runtime']['drive_contacts'];self.assertTrue(cfg['enabled']);self.assertEqual([{'joint':'replacement_joint','friction':.81}],cfg['wheel_friction']);self.assertFalse(m.verify_receipt(q)['ready'])
    def test_removing_multiple_missing_grips_reindexes_choices_and_keeps_other_wheels(self):
        xml='<robot name="wheels"><link name="base"><visual><geometry><box size=".3 .2 .06"/></geometry></visual></link>'
        parts=[]
        for i in range(3):
            part=f'<link name="wheel{i}"><visual><geometry><sphere radius="{.04+i*.01}"/></geometry></visual></link><joint name="joint{i}" type="continuous"><parent link="base"/><child link="wheel{i}"/><origin xyz="{i*.1} .2 0"/></joint><transmission name="tx{i}"><joint name="joint{i}"/><actuator name="motor{i}"/></transmission>'
            parts.append(part)
        self.zip(xml+''.join(parts)+'</robot>');p=self.load();data=m.read(p)
        data['runtime']['drive_contacts']['wheel_friction']=[{'joint':f'joint{i}','friction':.5+i*.1} for i in range(3)]
        for i in range(3):data['provenance'][f'runtime/drive_contacts/wheel_friction/{i}/friction']=f'measured wheel {i}'
        m.atomic_json(p,data);saved=self.ready(p);self.zip(xml+parts[2]+'</robot>');q=self.load(reuse=saved)
        for _ in range(2):
            pending=m.read(q)['migration']['pending'];i=next(i for i,item in enumerate(pending) if item.get('runtime_path'))
            self.assertEqual(['Remove saved wheel grip override'],pending[i]['options']);m.resolve_migration(q,i,'Remove saved wheel grip override')
        profile=m.read(q);cfg=profile['runtime']['drive_contacts'];self.assertTrue(cfg['enabled']);self.assertEqual([{'joint':'joint2','friction':.7}],cfg['wheel_friction'])
        self.assertEqual('measured wheel 2',profile['provenance']['runtime/drive_contacts/wheel_friction/0/friction']);self.assertNotIn('runtime/drive_contacts/wheel_friction/1/friction',profile['provenance'])
        pending=profile['migration']['pending'];i=next(i for i,item in enumerate(pending) if item.get('removed'));m.resolve_migration(q,i,'Acknowledge removed parts');self.assertFalse(m.verify_receipt(q)['ready'])
    def test_invalid_wheel_grip_settings_are_rejected(self):
        saved,_=self.measured_robot();p=m.fork_draft(saved,self.library);original=m.read(p)
        for value in ['bad',[{'joint':'wheel_joint','friction':True}],[{'joint':'wheel_joint','friction':-1}],[{'joint':'wheel_joint','friction':2.1}],[{'joint':'missing','friction':.6}],[{'joint':'wheel_joint','friction':.6}]*2]:
            data=copy.deepcopy(original);data['runtime']['drive_contacts']['wheel_friction']=value;m.atomic_json(p,data)
            with self.assertRaises(ValueError):m.compile_profile(p)
    def wheel_evidence(self):
        readings=[{'trial':f'trial{i}','set':'fit' if i<3 else 'validate','normal_load_n':n,'sliding_force_n':f} for i,(n,f) in enumerate([(10.,4.5),(20.,9.),(30.,13.5),(15.,6.75),(25.,11.25)])]
        csv='joint,trial,set,normal_load_n,sliding_force_n\n'+''.join(f'"wheel_joint","{r["trial"]}",{r["set"]},{r["normal_load_n"]},{r["sliding_force_n"]}\n' for r in readings)
        recording_hash=m.hashlib.sha256(csv.encode()).hexdigest();context=m.hashlib.sha256((recording_hash+'\n'+json.dumps('Physical test surface')+'\n0.6\n3,2,0.2,0.15,0.25').encode()).hexdigest()
        return {'schema_version':1,'method':'restrained-wheel sliding onset','recorded_joint':'wheel_joint','reference_surface':'Physical test surface','surface_friction':.6,'criteria':{'min_fit_trials':3,'min_validation_trials':2,'min_load_variation':.2,'max_relative_rmse':.15,'max_trial_relative_rmse':.25},'readings':readings,'recording_sha256':recording_hash,'fit_context_sha256':context,'result':{'wheel_coefficient':.75,'effective_mu':.45,'fit_trials':3,'validation_trials':2,'load_variation':2/3,'fit_rmse_n':0.,'validation_rmse_n':0.,'fit_relative_rmse':0.,'validation_relative_rmse':0.,'worst_trial_relative_rmse':0.}}
    def test_measured_grip_evidence_is_portable_and_migrates_with_its_wheel(self):
        saved,xml=self.measured_robot();p=m.fork_draft(saved,self.library);data=m.read(p);evidence=self.wheel_evidence();data['runtime']['drive_contacts']['wheel_friction']=[{'joint':'wheel_joint','friction':.75,'measurement':evidence}];data['provenance']['runtime/drive_contacts/wheel_friction/0/friction']='measured from physical trials';m.atomic_json(p,data);saved=self.ready(p);before=saved.read_bytes()
        bundle=self.root/'measured-grip.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable');self.assertEqual(evidence,m.read(restored)['runtime']['drive_contacts']['wheel_friction'][0]['measurement']);self.assertTrue(m.verify_receipt(restored)['ready'])
        self.zip(xml.replace('wheel_joint','renamed_joint'));changed=m.import_zip(self.package,self.root/'portable','robot',restored);grip=m.read(changed)['runtime']['drive_contacts']['wheel_friction'][0];self.assertEqual('renamed_joint',grip['joint']);self.assertEqual(evidence,grip['measurement']);self.assertFalse(m.verify_receipt(changed)['ready']);self.assertEqual(before,saved.read_bytes())
    def test_stale_measured_coefficient_and_cross_set_trials_cannot_compile(self):
        saved,_=self.measured_robot();p=m.fork_draft(saved,self.library);original=m.read(p)
        for change in ['coefficient','trial','result']:
            data=copy.deepcopy(original);evidence=self.wheel_evidence();grip={'joint':'wheel_joint','friction':.75,'measurement':evidence}
            if change=='coefficient':grip['friction']=.8
            if change=='trial':evidence['readings'][3]['trial']='trial0'
            if change=='result':evidence['result']['wheel_coefficient']=.9
            data['runtime']['drive_contacts']['wheel_friction']=[grip];m.atomic_json(p,data)
            with self.assertRaises(ValueError):m.compile_profile(p)
    def tire_evidence(self):
        spec={'static_mu':.9,'sliding_mu':.6,'lateral_scale':1.,'stiffness_n_per_mps':100.,'transition_mps':.15}
        readings=[]
        for i in range(18):
            slip=(-1 if i%2 else 1)*(.01+i*.03);normal=5.+i%3*5.
            force=math.copysign(min(100*abs(slip),(.6+.3*math.exp(-(slip/.15)**2))*normal),slip)
            readings.append({'recorded_joint':'wheel_joint','trial':f'trial{i}','set':'fit' if i<12 else 'validate','wheel_surface_mps':.3+slip,'hub_speed_mps':.3,'normal_load_n':normal,'longitudinal_force_n':force})
        # Backend metadata/transport fixture; full hash/fit checks are covered by native Java tests.
        evidence={'schema_version':1,'method':'steady direct-force longitudinal brush','tire_class':'traction','reference_surface':'Synthetic fixture','hub_speed_source':'Independent optical fixture','force_source':'Direct sensor fixture','context_sha256':'a'*64,'readings':readings,'bounds':{},'criteria':{},'result':{k:v for k,v in spec.items() if k!='lateral_scale'}}
        spec['measurement']=evidence
        return {'traction':spec,'omni':{'static_mu':.8,'sliding_mu':.5,'lateral_scale':.05,'stiffness_n_per_mps':80.,'transition_mps':.2},'omni_joints':[],'reflected_motor_inertia_kg_m2':.0015,'contact_tolerance_m':.004}
    def test_tire_evidence_transport_and_cad_migration_preserve_original_measurements(self):
        saved,xml=self.measured_robot();p=m.fork_draft(saved,self.library);data=m.read(p);tires=self.tire_evidence();tires['traction_response']={'lateral_stiffness_n_per_mps':70.,'lateral_mu':.4,'relaxation_time_s':.08};data['runtime']['tires']=tires;m.atomic_json(p,data);saved=self.ready(p);original=saved.read_bytes()
        bundle=self.root/'tire.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable');self.assertEqual(tires,m.read(restored)['runtime']['tires'])
        self.zip(xml.replace('wheel_joint','renamed_joint'));changed=m.import_zip(self.package,self.root/'portable','robot',restored);self.assertEqual(tires,m.read(changed)['runtime']['tires']);self.assertFalse(m.verify_receipt(changed)['ready']);self.assertEqual(original,saved.read_bytes())
    def test_tire_evidence_rejects_wrong_class_sign_leakage_and_manual_parameter_changes(self):
        saved,_=self.measured_robot();p=m.fork_draft(saved,self.library);original=m.read(p)
        for change in ['class','sign','partition','parameter']:
            data=copy.deepcopy(original);tires=self.tire_evidence();e=tires['traction']['measurement']
            if change=='class':e['tire_class']='omni'
            if change=='sign':e['readings'][0]['longitudinal_force_n']*=-1
            if change=='partition':e['readings'][-1]['trial']='trial0'
            if change=='parameter':tires['traction']['stiffness_n_per_mps']=120.
            data['runtime']['tires']=tires;m.atomic_json(p,data)
            with self.assertRaises(ValueError):m.compile_profile(p)
    def test_legacy_modes_do_not_change_on_capture_reimport_or_migration(self):
        p=self.load();data=m.read(p);data['runtime'].pop('drive_contacts');m.atomic_json(p,data);saved=self.ready(p);self.assertNotIn('drive_contacts',m.read(self.load())['runtime'])
        self.zip(URDF.replace('0 0 .06','0 0 .07'));self.assertNotIn('drive_contacts',m.read(self.load(reuse=saved))['runtime'])
        self.assertNotIn('drive_contacts',m.new_profile(p.parent/'source/model.urdf','robot',{})['runtime']);self.assertNotIn('drive_contacts',m.new_profile(p.parent/'source/model.urdf','field')['runtime'])
    def test_chassis_proxy_does_not_hide_a_powered_wheel_or_its_fixed_tread(self):
        xml=URDF.replace('<visual><geometry><box size="1 1 .02"/></geometry></visual>','<collision><geometry><box size="1 1 .02"/></geometry></collision>')
        xml=xml.replace('type="fixed"','type="continuous"').replace('</robot>','<transmission name="drive"><joint name="ball_mount"/><actuator name="left_front_drive"/></transmission></robot>');self.zip(xml)
        p=self.load();self.assertEqual('visual',m.read(p)['entities']['ball']['settings']['collision_strategy'])
        xml=xml.replace('<link name="ball"><visual><geometry><sphere radius=".04"/></geometry></visual></link>','<link name="ball"/><link name="tread"><visual><geometry><sphere radius=".04"/></geometry></visual></link><joint name="tread_mount" type="fixed"><parent link="ball"/><child link="tread"/></joint>');self.zip(xml)
        p=m.import_zip(self.package,self.library,'robot',fresh=True);self.assertEqual('visual',m.read(p)['entities']['tread']['settings']['collision_strategy'])
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
    def test_motion_review_metadata_is_portable_without_changing_physical_or_collision_digest(self):
        p=self.load();r=m.compile_profile(p);data=m.read(p);review={'schema_version':1,'entries':{'drive/forward':{'assessment':'Correct','model_digest':r['effective_digest'],'context':'hardware','run_id':'run','observation':{'forward_m':.2}}}}
        data['motion_review']=review;m.atomic_json(p,data);self.assertEqual(r['effective_digest'],m.compile_profile(p)['effective_digest']);saved=self.ready(p);original=saved.read_bytes();bundle=self.root/'motion.zip';m.export_bundle(saved,bundle);restored=m.import_bundle(bundle,self.root/'portable');self.assertEqual(review,m.read(restored)['motion_review']);self.assertTrue(m.verify_receipt(restored)['ready']);self.assertEqual(original,saved.read_bytes())
    def test_changed_cad_keeps_motion_assessments_as_history_with_the_old_digest(self):
        p=self.load();data=m.read(p);old=m.effective_digest(data);data['motion_review']={'schema_version':1,'entries':{'drive/forward':{'assessment':'Correct','model_digest':old,'context':'old hardware'}}};m.atomic_json(p,data);saved=self.ready(p);self.zip(URDF.replace('0 0 .06','0 0 .08'));changed=self.load(reuse=saved);self.assertEqual(m.read(saved)['motion_review'],m.read(changed)['motion_review']);self.assertNotEqual(old,m.verify_receipt(changed)['effective_digest']);self.assertFalse(m.verify_receipt(changed)['ready'])
    def test_manually_retained_part_is_not_reported_removed(self):
        saved=self.ready(self.load());self.zip(URDF.replace('1 1 .02','2 1 .02'));q=self.load(reuse=saved);data=m.read(q);item=data['migration']['pending'][0];choice=next(c for c in item['options'] if c.startswith('Use settings: '));old=choice[14:];m.resolve_migration(q,0,choice);pending=m.read(q)['migration']['pending'];self.assertFalse(any(old in item.get('removed',[]) for item in pending));self.assertFalse(m.verify_receipt(q)['ready']);self.assertTrue(m.verify_receipt(saved)['ready'])
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

    def measured_robot(self, mesh=False):
        wheel = '<mesh filename="wheel.stl"/>' if mesh else '<sphere radius=".04"/>'
        xml = f'''<robot name="Measured robot"><link name="base"><visual><geometry><box size=".3 .2 .1"/></geometry></visual></link>
        <link name="wheel~left"><visual><geometry>{wheel}</geometry></visual></link>
        <joint name="wheel_joint" type="continuous"><parent link="base"/><child link="wheel~left"/><origin xyz="0 .2 0"/><axis xyz="0 1 0"/></joint>
        <transmission name="tx"><joint name="wheel_joint"/><actuator name="cad_motor"><mechanicalReduction>1</mechanicalReduction></actuator></transmission></robot>'''
        self.zip(xml)
        if mesh:
            stl = self.root/'wheel.stl'
            m.write_stl(stl, [((0.,0.,0.),(.08,0.,0.),(0.,.08,.02))])
            with zipfile.ZipFile(self.package,'a') as package:
                package.write(stl,'other/urdf/wheel.stl')
        p = self.load()
        data = m.read(p)
        data['runtime'].update(total_mass_kg=9.0718474,
            drive={'type':'differential','left_motor':'leftDrive','right_motor':'rightDrive',
                   'wheel_radius_m':.0508,'track_width_m':.4064,'left_shaft_sign':-1,'right_shaft_sign':1},
            calibration_data={'battery':{'v_internal':12.7,'r_battery':.15},
                              'motors':{'leftDrive':{'tau_static_nm':.03,'viscous_b_nm_s_per_rad':.004}}})
        data['entities']['wheel~left']['settings']['actuators'] = [{'name':'leftDrive','mechanicalReduction':-2.}]
        data['provenance'].update({'runtime/total_mass_kg':'measured manually: operating robot with battery',
            'runtime/drive/wheel_radius_m':'measured manually: tread diameter',
            'runtime/drive/track_width_m':'measured manually: left/right wheel center spacing',
            'entities/wheel~0left/settings/actuators/0/name':'user supplied',
            'entities/wheel~0left/settings/actuators/0/mechanicalReduction':'measured manually: sprocket ratio'})
        m.atomic_json(p,data)
        return self.ready(p), xml

    def assert_preserved(self, expected, path, part='wheel~left'):
        data = m.read(path)
        self.assertEqual(expected['runtime'], data['runtime'])
        for key,value in expected['provenance'].items():
            if key.startswith('runtime/'):
                self.assertEqual(value, data['provenance'][key])
        self.assertEqual(expected['entities']['wheel~left']['settings']['actuators'],
                         data['entities'][part]['settings']['actuators'])
        base = 'entities/'+m.pointer(part)+'/settings/actuators/0/'
        self.assertEqual('user supplied',data['provenance'][base+'name'])
        self.assertEqual('measured manually: sprocket ratio',data['provenance'][base+'mechanicalReduction'])
        prepared = m.xml(path.parent/'prepared/robot.urdf').find('transmission/actuator')
        self.assertEqual('leftDrive',prepared.get('name'))
        self.assertEqual(-2.,float(prepared.findtext('mechanicalReduction')))
        self.assertEqual(expected['runtime']['calibration_data'], m.read(path.parent/'prepared/calibration.json'))

    def test_measured_urdf_stl_bundle_preserves_bindings_and_calibration_in_another_library(self):
        saved,_ = self.measured_robot(mesh=True)
        before = saved.read_bytes()
        bundle = self.root/'measured.zip'
        m.export_bundle(saved,bundle)
        restored = m.import_bundle(bundle,self.root/'portable')
        self.assert_preserved(m.read(saved),restored)
        self.assertTrue(m.verify_receipt(restored)['ready'])
        self.assertEqual(m.read(saved)['provenance'],m.read(restored)['provenance'])
        self.assertTrue(list((restored.parent/'source/assets').glob('*.stl')))
        self.assertEqual(before,saved.read_bytes())

    def test_automatic_cad_rename_preserves_measurements_and_moves_binding_provenance(self):
        saved,xml = self.measured_robot()
        original = saved.read_bytes()
        bundle = self.root/'measured.zip'
        m.export_bundle(saved,bundle)
        restored = m.import_bundle(bundle,self.root/'portable')
        self.zip(xml.replace('wheel~left','wheel~front').replace('0 .2 0','0 .22 0'))
        migrated = m.import_zip(self.package,self.root/'portable','robot',restored)
        self.assert_preserved(m.read(saved),migrated,'wheel~front')
        self.assertEqual(m.read(saved)['entities']['wheel~left']['id'],m.read(migrated)['entities']['wheel~front']['id'])
        self.assertFalse(any(key.startswith('entities/wheel~0left/') for key in m.read(migrated)['provenance']))
        self.assertFalse(m.verify_receipt(migrated)['ready'])
        self.assertEqual([],m.read(migrated)['migration']['pending'])
        self.assertRaisesRegex(ValueError,'preview|validateModel',m.save_revision,migrated,self.root/'portable',True)
        self.assertEqual(original,saved.read_bytes())

    def test_changed_geometry_retained_settings_survive_migration_and_second_round_trip(self):
        saved,xml = self.measured_robot()
        original = saved.read_bytes()
        self.zip(xml.replace('radius=".04"','radius=".06"'))
        migrated = self.load(reuse=saved)
        pending = m.read(migrated)['migration']['pending']
        i = next(i for i,item in enumerate(pending) if item.get('new')=='wheel~left')
        self.assertIn('Use settings: wheel~left',pending[i]['options'])
        self.assertRaisesRegex(ValueError,'migration',m.save_revision,migrated,self.library,True)
        m.resolve_migration(migrated,i,'Use settings: wheel~left')
        self.assert_preserved(m.read(saved),migrated)
        self.assertFalse(m.verify_receipt(migrated)['ready'])
        self.assertEqual([],m.read(migrated)['migration']['pending'])
        updated = self.ready(migrated)
        bundle = self.root/'migrated.zip'
        m.export_bundle(updated,bundle)
        restored = m.import_bundle(bundle,self.root/'after-migration')
        self.assert_preserved(m.read(saved),restored)
        self.assertTrue(m.verify_receipt(restored)['ready'])
        self.assertEqual(original,saved.read_bytes())

    def test_changed_cad_binding_requires_choice_and_attributes_replacement_to_cad(self):
        saved,xml = self.measured_robot()
        self.zip(xml.replace('cad_motor','new_cad_motor'))
        for choice in ['Keep saved bindings','Use new CAD bindings']:
            with self.subTest(choice=choice):
                migrated = self.load(reuse=saved)
                pending = m.read(migrated)['migration']['pending']
                i = next(i for i,item in enumerate(pending) if item.get('binding_link'))
                self.assertEqual(['Keep saved bindings','Use new CAD bindings'],pending[i]['options'])
                self.assertFalse(m.verify_receipt(migrated)['ready'])
                m.resolve_migration(migrated,i,choice)
                if choice=='Keep saved bindings':
                    self.assert_preserved(m.read(saved),migrated)
                else:
                    data = m.read(migrated)
                    self.assertEqual([{'name':'new_cad_motor','mechanicalReduction':1.}],data['entities']['wheel~left']['settings']['actuators'])
                    base = 'entities/wheel~0left/settings/actuators'
                    self.assertEqual('source CAD',data['provenance'][base])
                    self.assertFalse(any(key.startswith(base+'/') for key in data['provenance']))
                    self.assertEqual(m.read(saved)['runtime'],data['runtime'])

    def test_ambiguous_renamed_part_keeps_measurement_provenance_after_user_choice(self):
        saved,xml = self.measured_robot()
        extra = '''<link name="duplicate"><visual><geometry><sphere radius=".04"/></geometry></visual></link>
        <joint name="duplicate_joint" type="fixed"><parent link="base"/><child link="duplicate"/><origin xyz="0 -.2 0"/></joint>'''
        self.zip(xml.replace('</robot>',extra+'</robot>'))
        with_duplicate = self.load(reuse=saved)
        pending = m.read(with_duplicate)['migration']['pending']
        i = next(i for i,item in enumerate(pending) if item.get('new')=='duplicate')
        m.resolve_migration(with_duplicate,i,'Generate defaults')
        saved = self.ready(with_duplicate)
        original = saved.read_bytes()
        self.zip(xml.replace('</robot>',extra+'</robot>').replace('wheel~left','wheel~front'))
        migrated = self.load(reuse=saved)
        pending = m.read(migrated)['migration']['pending']
        i = next(i for i,item in enumerate(pending) if item.get('new')=='wheel~front')
        self.assertIn('Use settings: wheel~left',pending[i]['options'])
        m.resolve_migration(migrated,i,'Use settings: wheel~left')
        self.assert_preserved(m.read(saved),migrated,'wheel~front')
        self.assertFalse(any(key.startswith('entities/wheel~0left/') for key in m.read(migrated)['provenance']))
        self.assertEqual([],m.read(migrated)['migration']['pending'])
        self.assertFalse(m.verify_receipt(migrated)['ready'])
        self.assertEqual(original,saved.read_bytes())

    def test_rename_into_removed_part_name_does_not_take_that_parts_old_labels(self):
        saved,xml = self.measured_robot()
        extra = '''<link name="duplicate"><visual><geometry><sphere radius=".07"/></geometry></visual></link>
        <joint name="duplicate_joint" type="fixed"><parent link="base"/><child link="duplicate"/><origin xyz="0 -.2 0"/></joint>'''
        self.zip(xml.replace('</robot>',extra+'</robot>'))
        with_duplicate = self.load(reuse=saved)
        pending = m.read(with_duplicate)['migration']['pending']
        i = next(i for i,item in enumerate(pending) if item.get('new')=='duplicate')
        m.resolve_migration(with_duplicate,i,'Generate defaults')
        data = m.read(with_duplicate)
        data['provenance']['entities/duplicate/settings/actuators/0/mechanicalReduction'] = 'unrelated old part ratio'
        m.atomic_json(with_duplicate,data)
        saved = self.ready(with_duplicate)
        self.zip(xml.replace('wheel~left','duplicate'))
        migrated = self.load(reuse=saved)
        self.assert_preserved(m.read(saved),migrated,'duplicate')
        self.assertFalse(m.verify_receipt(migrated)['ready'])
        self.assertTrue(any('duplicate' in item.get('removed',[]) for item in m.read(migrated)['migration']['pending']))
if __name__=='__main__':unittest.main()
