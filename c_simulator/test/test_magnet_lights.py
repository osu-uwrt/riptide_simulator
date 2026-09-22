import sys
from pathlib import Path
import unittest
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from magnet_light_model import MagnetLights, pose


class MagnetLightTest(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/talos_tasks.yaml').read_text())['magnet_lights']
        self.frames={'magnet_target1':pose([1,2,-1,0,0,.7]),
                     'magnet_target2':pose([-1,2,-1,0,0,-1.2])}
        self.vehicle={'base_link':[-.14,.03,-.09],
                      'magnet':{'pose':[-.2946,.0475,-.391,0,0,0]}}
        self.model=MagnetLights(self.cfg,self.frames,self.vehicle)

    def body_near(self,key='magnet_target1',distance=.1):
        body=pose([0,0,0,.15,-.25,.8])
        target=self.model.sensors[key]+self.model.faces[key][:3,0]*distance
        body[:3,3]=target-body[:3,:3]@self.model.tip
        return body

    def test_exact_six_inch_boundary_and_half_second_dwell(self):
        body=self.body_near(distance=.1524-1e-9)
        self.assertEqual(self.model.step(.499,body),[])
        self.assertEqual(self.model.states['magnet_target1'],'red')
        self.assertEqual(self.model.step(.001,body),['magnet_target1'])
        self.assertEqual(self.model.states['magnet_target2'],'red')
        self.assertEqual(self.model.step(10,self.body_near(distance=2)),[])
        self.assertEqual(self.model.states['magnet_target1'],'green')
        self.model.reset()
        self.assertEqual(self.model.step(1,self.body_near(distance=.1524+1e-6)),[])

    def test_leave_range_or_lose_truth_restarts_dwell(self):
        near=self.body_near()
        for away in (None,self.body_near(distance=.2)):
            self.model.reset()
            self.model.step(.4,near);self.model.step(.1,away)
            self.assertEqual(self.model.step(.1,near),[])
            self.assertEqual(self.model.step(.4,near),['magnet_target1'])

    def test_magnet_tip_and_tilted_sensor_follow_frames(self):
        np.testing.assert_allclose(self.model.tip,[-.1546,.0175,-.251])
        for key in self.frames:
            self.assertAlmostEqual(self.model.faces[key][2,0],np.sqrt(.5))
            self.assertAlmostEqual(np.linalg.norm(self.model.sensors[key]-self.frames[key][:3,3]),.008)
        # Base at the light is insufficient: the magnet is over 29 cm away.
        body=np.eye(4);body[:3,3]=self.model.sensors['magnet_target1']
        self.assertEqual(self.model.step(1,body),[])

    def test_independent_targets_initial_green_and_reset(self):
        self.cfg['targets']['magnet_target2']='green'
        m=MagnetLights(self.cfg,self.frames,self.vehicle)
        self.assertEqual(m.step(.5,self.body_near()),['magnet_target1'])
        m.reset()
        self.assertEqual(m.states,{'magnet_target1':'red','magnet_target2':'green'})
        self.assertEqual(m.step(.2,self.body_near()),[])

    def test_timestep_independence_and_pause(self):
        for dt in (.002,.01,.1):
            self.model.reset();near=self.body_near('magnet_target2')
            for _ in range(round(.5/dt)-1):self.assertEqual(self.model.step(dt,near),[])
            for _ in range(20):self.assertEqual(self.model.step(0,near),[])
            self.assertEqual(self.model.step(dt,near),['magnet_target2'])

    def test_bad_configuration_and_time_rejected(self):
        for key,value in [('trigger_distance',float('nan')),('trigger_distance',-.1),
                          ('activation_time',.6),('activation_time',0),('targets',{'magnet_target1':'blue'})]:
            cfg=dict(self.cfg);cfg[key]=value
            with self.assertRaises(ValueError):MagnetLights(cfg,self.frames,self.vehicle)
        for dt in (-1,float('nan')):
            with self.assertRaises(ValueError):self.model.step(dt,self.body_near())


if __name__=='__main__':unittest.main()
