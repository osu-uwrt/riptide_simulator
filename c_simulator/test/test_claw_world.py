#!/usr/bin/env python3
"""Physical pick/carry/release regressions against the actual course meshes."""
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
import pybullet as pb
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from claw_world import ClawWorld, pose_matrix


class ClawContactTest(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((ROOT/'config/talos_tasks.yaml').read_text())['claw']
        data=yaml.safe_load((ROOT.parent/'config.yaml').read_text())['/talos/riptide_mapping2']['ros__parameters']['init_data']
        self.frames={}
        def resolve(k):
            k=k.removesuffix('_frame')
            if k in ('map','world'):return np.eye(4)
            if k not in self.frames:
                a=data[k];v=a['pose']
                self.frames[k]=resolve(a['parent'])@pose_matrix([v.get(c,0) for c in ('x','y','z')],pb.getQuaternionFromEuler([0,0,np.deg2rad(v.get('yaw',0))]))
            return self.frames[k]
        for key in ('table','helmet','warning',*self.cfg['props']):resolve(key)
        self.w=ClawWorld(self.cfg,self.frames,np.eye(4),ROOT/'collision_files/tasks',np.eye(4))
        self.body=np.eye(4);self.body[:3,3]=[0,0,1]
        self.step(.8)

    def tearDown(self):self.w.close()
    def step(self,seconds,enabled=True,velocity=None):
        for _ in range(round(seconds/.002)):
            self.w.step(.002,self.body,np.zeros(3) if velocity is None else velocity,
                        np.zeros(3),np.zeros(3),998.2,enabled)
    def move(self,destination,seconds=1.5):
        start=self.body[:3,3].copy(); n=round(seconds/.002)
        velocity=(destination-start)/seconds
        for i in range(n):
            self.body[:3,3]=start+(destination-start)*(i+1)/n
            self.step(.002,velocity=velocity)
    def grasp(self,key):
        self.w.command(True);self.step(2.4)
        p=self.w.props[key];t=self.w.prop_pose(key)
        self.body[:3,:3]=np.eye(3)
        target=t[:3,3]+[0,0,2*p['center'][2]+.003]
        if key=='nut_and_bolt':
            # At the table corner, full-open pads hit the raised edge. Approach
            # diagonally with a narrower gap, as a physical claw must.
            self.body[:3,:3]=self.frames['table'][:3,:3]@pose_matrix([0,0,0],pb.getQuaternionFromEuler([0,0,np.pi/6]))[:3,:3]
            target+=np.array([0,0,.012])-self.body[:3,:3]@np.array([.045,0,0])
            self.body[:3,3]=target+[0,0,.2]
            self.w.command(False,duration=(self.w.travel-.037)/self.cfg['jaw_speed']);self.step(1.25)
            self.move(target)
        elif key=='plug':
            # Avoid sweeping an open jaw through the adjacent corner post.
            self.body[:3,:3]=self.frames['table'][:3,:3]
            self.body[:3,3]=target+[0,0,.2]
            self.w.command(False,duration=(self.w.travel-.045)/self.cfg['jaw_speed']);self.step(.9)
            self.move(target)
        else:self.body[:3,3]=target
        if key not in ('nut_and_bolt','plug'):self.step(.1)
        self.w.command(False);self.step(2.4)
        self.assertEqual(self.w.held,key,self.w.events)
        if key=='nut_and_bolt':
            # Clear the raised corner post before lifting the wide pad backs.
            self.move(self.body[:3,3]+self.frames['table'][:3,:3]@np.array([-.02,-.02,.005]),1.)
            self.assertEqual(self.w.held,key,self.w.events)
    def test_all_four_grasp_lift_and_deliver(self):
        for key in self.w.props:
            self.w.reset();self.body[:3,3]=[0,0,1];self.step(.8)
            self.grasp(key)
            before=self.w.prop_pose(key)[:3,3].copy()
            self.move(self.body[:3,3]+[0,0,.25])
            self.assertEqual(self.w.held,key)
            self.assertGreater(self.w.prop_pose(key)[2,3]-before[2],.23)
            self.assertFalse(any(e[1]=='slipped' for e in self.w.events))
            basket=self.w.props[key]['config']['basket']
            self.move(self.frames[basket][:3,3]+[0,0,.25],3.)
            self.w.command(True);self.assertIsNone(self.w.held)
            self.step(3.)
            self.assertIn((key,'success',basket),self.w.events)
    def test_correct_basket_and_surface(self):
        self.grasp('pill');self.move(self.body[:3,3]+[0,0,.3])
        self.move(np.array([self.body[0,3],self.body[1,3],.1]),3.)
        self.assertIn(('pill','surfaced',''),self.w.events)
        destination=self.frames['helmet'][:3,3]+[0,0,.25]
        self.move(destination,3.);self.w.command(True);self.step(3.)
        self.assertEqual(self.w.events.count(('pill','success','helmet')),1)
        self.step(1.);self.assertEqual(self.w.events.count(('pill','success','helmet')),1)
    def test_wrong_basket(self):
        self.grasp('pill');self.move(self.body[:3,3]+[0,0,.3])
        self.move(self.frames['warning'][:3,3]+[0,0,.25]);self.w.command(True);self.step(3.)
        self.assertIn(('pill','wrong_target','warning'),self.w.events)
    def test_miss_floor_and_reset(self):
        self.grasp('pill');self.move(self.body[:3,3]+[0,0,.3])
        self.move(self.body[:3,3]+[1,0,0]);self.w.command(True);self.step(4.)
        self.assertIn(('pill','miss','pool_floor'),self.w.events)
        self.assertGreaterEqual(self.w.prop_pose('pill')[2,3],-2.134)
        self.w.reset();self.assertIsNone(self.w.held);self.assertFalse(self.w.events)
        np.testing.assert_allclose(self.w.prop_pose('pill'),self.frames['pill'],atol=1e-7)
    def test_empty_close_and_disarm(self):
        np.testing.assert_allclose(self.w.joints(),[0.,0.],atol=1e-12)
        self.w.command(True);self.step(2.4)
        self.w.command(False);self.step(2.4)
        self.assertIsNone(self.w.held);self.assertFalse(self.w.events)
        self.w.command(True);self.step(.2);before=self.w.joints()
        self.step(1.,enabled=False)
        np.testing.assert_allclose(self.w.joints(),before,atol=.003)
    def test_reset_restores_contact_state(self):
        expected={k:self.w.prop_pose(k).copy() for k in self.w.props}
        self.grasp('pill');self.move(self.body[:3,3]+[0,0,.3])
        self.w.reset();self.body=np.eye(4);self.body[:3,3]=[0,0,1];self.step(.8)
        np.testing.assert_allclose(self.w.joints(),[0.,0.],atol=1e-12)
        for key,t in expected.items():
            np.testing.assert_allclose(self.w.prop_pose(key),t,atol=1e-6,err_msg=key)

    def test_jaws_translate_without_rotating(self):
        for q in (0.,self.w.travel/2,self.w.travel):
            self.w._place_pads(self.body,q,np.zeros(3),np.zeros(3),0)
            for uid,sign in zip(self.w.pads,(1,-1)):
                pos,orientation=self.w.b.getBasePositionAndOrientation(uid)
                np.testing.assert_allclose(pos,self.body[:3,3]+[0,sign*q,0],atol=1e-7)
                np.testing.assert_allclose(orientation,[0,0,0,1],atol=1e-7)

    def test_table_impact_does_not_backdrive_jaws(self):
        self.w.q=self.w.target=.04
        self.body=self.frames['table'].copy();self.body[2,3]-=.003
        self.step(.5,velocity=np.array([0,0,-.4]))
        np.testing.assert_allclose(self.w.joints(),[.04,.04],atol=1e-12)
        for uid in self.w.pads:
            pos,rot=self.w.b.getBasePositionAndOrientation(uid)
            self.assertTrue(np.isfinite(pos).all());self.assertTrue(np.isfinite(rot).all())

    def test_table_is_solid_under_prop_pressure(self):
        p=self.w.props['pill'];t=self.frames['table'];position=(t@np.array([0,0,.15,1]))[:3]
        self.w.b.resetBasePositionAndOrientation(p['id'],position,[0,0,0,1])
        for _ in range(1000):
            xyz,_=self.w.b.getBasePositionAndOrientation(p['id'])
            self.w.b.applyExternalForce(p['id'],-1,[0,0,-20],xyz,pb.WORLD_FRAME)
            self.step(.002)
            self.assertGreater(xyz[2]-p['half'][2],t[2,3]-.002)

    def test_basket_wall_blocks_side_entry(self):
        p=self.w.props['pill'];t=self.frames['helmet']
        # Put a released prop alongside the basket; drive it sideways at wall height.
        pos=(t@np.array([.15,0,.035,1]))[:3]
        self.w.b.resetBasePositionAndOrientation(p['id'],pos,[0,0,0,1])
        self.w.b.resetBaseVelocity(p['id'],-t[:3,0]*.3,[0,0,0])
        self.step(.3)
        xyz,_=self.w.b.getBasePositionAndOrientation(p['id'])
        local=t[:3,:3].T@(np.asarray(xyz)-t[:3,3])
        self.assertGreater(local[0],.09)
        self.assertFalse(any(e[1]=='success' for e in self.w.events))

if __name__=='__main__':unittest.main()
