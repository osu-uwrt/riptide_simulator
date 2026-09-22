#!/usr/bin/env python3
"""Profile composition, validation, install portability, and year-owned scoring."""
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from riptide_sim_config.profiles import resolve, read, validate_frames, load_behavior
from ament_index_python.packages import get_package_share_directory


class Profiles(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)/'installed'
        original=Path(__file__).resolve().parents[1]
        shutil.copytree(original,self.root,ignore=shutil.ignore_patterns('__pycache__'))
        self.output=Path(self.temp.name)/'runs'

    def resolve(self,*args,**kwargs):
        return resolve(*args,root=self.root,output=self.output,**kwargs)

    def edit(self,relative,callback):
        path=self.root/relative
        value=read(path);callback(value);path.write_text(yaml.safe_dump(value))

    def test_independent_selections_and_world(self):
        for robot,year,scenario in [('talos','2026','default'),('talos','example','default'),('example_auv','example','default'),('example_auv','2026','empty_pool')]:
            path=self.resolve(robot,year,scenario)
            meta=read(path/'selection.yaml');vehicle=read(path/'vehicle.yaml')
            self.assertEqual((meta['robot'],meta['year']),(robot,year))
            self.assertEqual(meta['world'],read(path/'task.yaml')['world'])
            self.assertEqual(meta['world'],read(path/'scene.yaml')['world'])
            self.assertEqual(meta['world']['water_level'],read(path/'hydrodynamics.yaml')['water_level'])
            if robot=='example_auv':
                self.assertNotIn('status_lights_config',meta['viewer'])
                self.assertNotIn('thruster_visuals_config',meta['viewer'])
                self.assertEqual(len(vehicle['thrusters']),4)
                self.assertEqual([c['name'] for c in vehicle['sim_cameras']],['survey'])
                self.assertEqual(vehicle['sim_enabled_sensors'],[])
            if year=='example':
                self.assertNotIn('claw',read(path/'task.yaml'))
                self.assertNotIn('torpedo',read(path/'task.yaml'))
            if scenario=='empty_pool':
                self.assertFalse(meta['enabled']);self.assertEqual(read(path/'task.yaml')['behavior'],'')
                self.assertEqual(read(path/'scene.yaml')['objects'],{})
        self.assertEqual(self.resolve(),self.resolve())

    def test_unknown_and_incompatible(self):
        for args in [('missing','2026'),('talos','missing'),('talos','2026','missing'),('example_auv','2026')]:
            with self.assertRaises(ValueError):self.resolve(*args)

    def test_bad_mass_camera_frame_and_world(self):
        self.edit('robots/example_auv/config/vehicle.yaml',lambda c:c.update(mass=float('nan')))
        with self.assertRaisesRegex(ValueError,'mass'):self.resolve('example_auv','example')
        for data in ({'a':{'parent':'missing'}},{'a':{'parent':'b'},'b':{'parent':'a'}}):
            with self.assertRaises(ValueError):validate_frames(data)

    def test_zero_cameras_and_namespace(self):
        self.edit('robots/example_auv/robot.yaml',lambda c:c.update(cameras=[]))
        path=self.resolve('example_auv','example',overrides={'namespace':'test_sub'})
        self.assertEqual(read(path/'vehicle.yaml')['sim_cameras'],[])
        self.assertEqual(read(path/'hydrodynamics.yaml')['robot'],'test_sub')
        self.assertIn('/test_sub/riptide_mapping2',read(path/'mapping.yaml'))

    def test_year_validator(self):
        self.edit('tasks/example/config/tasks.yaml',lambda c:c['regions'].append(c['regions'][0]))
        with self.assertRaisesRegex(ValueError,'unique'):self.resolve('example_auv','example')

    def test_scenario_defaults_and_scoring(self):
        self.edit('tasks/example/scenarios/default.yaml',lambda c:c.update(scoring_overrides={'region_points':777},runtime={'random_seed':42}))
        path=self.resolve('example_auv','example')
        self.assertEqual(read(path/'task.yaml')['scoring_rules']['region_points'],777)
        self.assertEqual(read(path/'selection.yaml')['runtime']['random_seed'],42)
        rules=read(self.root/'tasks/2026/config/scoring.yaml')
        cls=load_behavior(str(self.root/'tasks/2026/behavior/scoring.py')+':RunScore')
        score=cls(rules)
        self.assertEqual(score.rules['gate'],100)
        rules['points']['gate']=321
        changed=cls(rules)
        self.assertEqual(changed.rules['gate'],321)
        changed.start(0);changed.gate('repair',-1)
        self.assertEqual(changed.snapshot(1)['total'],321)

    def test_install_assets_and_coin_defaults(self):
        # Physically copied profile tree; no __file__ traversal back into src.
        path=self.resolve()
        task=read(path/'task.yaml')
        self.assertTrue(Path(task['robot_collision']).is_file())
        lights=read(read(path/'selection.yaml')['viewer']['status_lights_config'])
        self.assertEqual(lights['input']['topic'],'command/led')
        self.assertEqual(len(lights['lights']),3)
        rotors=read(read(path/'selection.yaml')['viewer']['thruster_visuals_config'])
        self.assertEqual(rotors['topic'],'simulator/actual_thruster_forces')
        self.assertEqual([r['input_index'] for r in rotors['rotors']],list(range(8)))
        self.assertTrue(Path(task['behavior'].rsplit(':',1)[0]).is_relative_to(self.root))
        defaults={o['key']:o['default'] for o in task['ui']['run_options']}
        self.assertTrue(defaults['heading_coin']);self.assertTrue(defaults['role_coin'])

if __name__=='__main__':unittest.main()
