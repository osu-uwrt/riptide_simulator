import sys
from pathlib import Path
import unittest
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from payload_model import advance, advance_rotating, crossing, torpedo_contact, launch_speed, effective_mass, support_extent, payload_mounts


class PayloadModel(unittest.TestCase):
    def setUp(self):
        self.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/talos_tasks.yaml').read_text())
    def test_loaded_centers_remain_seated_in_cad_mechanism(self):
        # CAD centers are independent of the TF calibration/aiming origins.
        vehicle = dict(base_link=[-.14, .03, -.09],
                       torpedoes=dict(pose=[.048, .18200625, -.1324, 0, 0, 0], baseline=.0365125),
                       droppers=dict(pose=[0, .17200625, -.1501, 0, 0, 0]))
        centers = {'torpedo': [[.06394921, .15639983, -.15425550],
                              [.06394921, .19140310, -.15425550]],
                   'dropper': [[-.04337956, .15639983, -.27306563],
                               [-.04337956, .19140310, -.27306563]]}
        for kind, expected in centers.items():
            mounts = payload_mounts(vehicle, self.cfg, kind)
            np.testing.assert_allclose([m[:3, 3] + vehicle['base_link'] for m in mounts], expected, atol=1e-9)

    def test_mounts_follow_rotated_robot_frames(self):
        self.cfg['torpedo']['slot_offsets'] = [[.03, 0, -.02, 0, 0, 0]] * 2
        self.cfg['dropper']['slot_offsets'] = [[0, -.02, 0, 0, np.pi/2, 0],
                                              [0, .02, 0, 0, np.pi/2, 0]]
        vehicle = dict(base_link=[.1, -.2, .3],
                       torpedoes=dict(pose=[1, 2, 3, 0, 0, np.pi/2], baseline=.04),
                       droppers=dict(pose=[1, 2, 3, 0, 0, np.pi/2]))
        torpedoes = payload_mounts(vehicle, self.cfg, 'torpedo')
        np.testing.assert_allclose(torpedoes[0][:3, 3], [.92, 2.23, 2.68])
        np.testing.assert_allclose(torpedoes[1][:3, 3], [.88, 2.23, 2.68])
        for mount in torpedoes:
            np.testing.assert_allclose(mount[:3, 0], [0, 1, 0], atol=1e-12)
        droppers = payload_mounts(vehicle, self.cfg, 'dropper')
        np.testing.assert_allclose(np.mean([m[:3, 3] for m in droppers], axis=0), [.9, 2.2, 2.7])
        for mount in droppers:
            np.testing.assert_allclose(mount[:3, 0], [0, 0, -1], atol=1e-12)
        # Tilting the physical mount must tilt the release axis too.
        vehicle['droppers']['pose'][3:] = [np.pi/2, 0, 0]
        for mount in payload_mounts(vehicle, self.cfg, 'dropper'):
            np.testing.assert_allclose(mount[:3, 0], [0, 1, 0], atol=1e-12)

    def test_three_slots_without_legacy_baseline(self):
        vehicle=dict(base_link=[0,0,0],torpedoes=dict(pose=[1,0,0,0,0,0]))
        self.cfg['torpedo'].update(count=3,slot_offsets=[[0,y,0,0,0,0] for y in (-.1,0,.1)])
        mounts=payload_mounts(vehicle,self.cfg,'torpedo')
        np.testing.assert_allclose([m[:3,3] for m in mounts],[[1,-.1,0],[1,0,0],[1,.1,0]])

    def test_water_level_translation_preserves_flight(self):
        cfg=dict(self.cfg['dropper'])
        position=np.array([0.,0.,-1.]);velocity=np.array([.2,0.,0.]);axis=np.array([0.,0.,-1.])
        p,v=advance(position,velocity,axis,cfg,np.zeros(3),998.2,.02)
        cfg['water_level']=4.
        shifted,w=advance(position+[0,0,4],velocity,axis,cfg,np.zeros(3),998.2,.02)
        np.testing.assert_allclose(shifted,p+[0,0,4]);np.testing.assert_allclose(w,v)

    def test_invalid_mounts_rejected(self):
        vehicle = dict(base_link=[0, 0, 0], torpedoes=dict(pose=[0]*6, baseline=float('nan')))
        with self.assertRaises(ValueError):
            payload_mounts(vehicle, self.cfg, 'torpedo')
        vehicle['torpedoes']['baseline'] = .04
        self.cfg['torpedo']['count'] = 3
        with self.assertRaises(ValueError):
            payload_mounts(vehicle, self.cfg, 'torpedo')

    def test_high_speed_hole_crossing_and_clearance(self):
        c=self.cfg['torpedo'];hole=c['holes'][0];yz=(np.array(hole['uv'])-.5)*2*c['panel_half_size']
        start=np.r_[1.,yz];end=np.r_[-1.,yz]
        hit=torpedo_contact(start,end,np.array([-1.,0,0]),c)
        self.assertEqual(hit[:3],('pass','fire_large','fire'))
        # A center inside the circular aperture is insufficient when the body clips its edge.
        yz[0]+=hole['radius_uv']*2*c['panel_half_size']-c['radius']/2
        hit=torpedo_contact(np.r_[1.,yz],np.r_[-1.,yz],np.array([-1.,0,0]),c)
        self.assertEqual(hit[0],'blocked')
    def test_no_repeated_crossing_or_outside_panel_hit(self):
        self.assertIsNone(crossing(np.array([0.,0,0]),np.array([1.,0,0]),0))
        self.assertIsNone(torpedo_contact(np.array([1.,1,0]),np.array([-1.,1,0]),np.array([-1.,0,0]),self.cfg['torpedo']))
    def test_marker_sinks_and_drag_opposes_flow_relative_motion(self):
        c=dict(self.cfg['dropper']);p=np.array([0.,0.,-1.]);v=np.zeros(3);axis=np.array([0.,0.,-1.])
        for _ in range(100):p,v=advance(p,v,axis,c,np.zeros(3),998.2,.002)
        self.assertLess(p[2],-1);self.assertLess(v[2],0)
        c['displaced_volume']=c['mass']/998.2;v=np.array([1.,.5,0]);p=np.array([0.,0.,-1.])
        _,after=advance(p,v,axis,c,np.zeros(3),998.2,.002)
        self.assertLess(np.linalg.norm(after),np.linalg.norm(v))
    def test_shared_exterior_slightly_negative_torpedo_and_solid_marker(self):
        torpedo,marker=self.cfg['torpedo'],self.cfg['dropper']
        for key in ('length','radius','displaced_volume','spring_energy','drag_axial','drag_lateral'):
            self.assertEqual(torpedo[key],marker[key])
        density=998.2;p=np.array([0.,0.,-1.]);v=np.zeros(3)
        for _ in range(500):p,v=advance(p,v,np.array([1.,0,0]),torpedo,np.zeros(3),density,.002)
        self.assertLess(p[2],-1)
        self.assertGreater(p[2],-1.1)
        self.assertGreater(effective_mass(torpedo,density),density*torpedo['displaced_volume'])
        self.assertLess(effective_mass(torpedo,density),1.03*density*torpedo['displaced_volume'])
        self.assertLess(effective_mass(torpedo,density),effective_mass(marker,density))
        self.assertGreater(launch_speed(torpedo,density),launch_speed(marker,density))
        self.assertGreater(launch_speed(marker,density),0)
        self.assertAlmostEqual(support_extent(np.array([0,0,-1]),marker)[2],marker['length']/2)

    def flight(self, cfg=None, dt=.002, duration=1., orientation=None, water=None, velocity=None):
        c=self.cfg['torpedo'] if cfg is None else cfg
        r=np.eye(3) if orientation is None else orientation.copy()
        p=np.array([0.,0.,-1.]);w=np.zeros(3)
        v=r[:,0]*launch_speed(c,998.2) if velocity is None else velocity.copy()
        water=np.zeros(3) if water is None else water
        samples=[]
        for _ in range(round(duration/dt)):
            p,v,r,w=advance_rotating(p,v,r,w,c,water,998.2,dt)
            samples.append((p.copy(),v.copy(),r.copy(),w.copy()))
        return samples

    def test_torpedo_flies_about_two_feet_then_dives(self):
        # Exercise the reference calibration even when the user adjusts local
        # damping for a straighter flight in the editable task configuration.
        samples=self.flight(cfg=dict(self.cfg['torpedo'],angular_damping=.006),duration=3.)
        p,v,r,w=next(s for s in samples if s[0][0]>=.6096)
        # A nearly straight first two feet (under one inch of drop), followed
        # by a pronounced nose-down descent as forward momentum decays.
        self.assertGreater(p[2],-1.025)
        self.assertLess(p[2],-1.)
        self.assertGreater(r[2,0],-np.sin(np.radians(15)))
        p,v,r,w=samples[-1]
        self.assertLess(p[2],-1.75)
        self.assertLess(r[2,0],-np.sin(np.radians(60)))
        self.assertLess(v[2],-.2)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r),1.)

    def test_balance_torque_direction_and_neutral_compatibility(self):
        c=dict(self.cfg['torpedo'],neutral_buoyancy=True)
        # Even a neutrally buoyant front-heavy body should pitch nose down.
        down=self.flight(cfg=c,duration=.1,velocity=np.zeros(3))[-1][2]
        self.assertLess(down[2,0],0.)
        c['center_of_buoyancy']=.04
        up=self.flight(cfg=c,duration=.1,velocity=np.zeros(3))[-1][2]
        self.assertGreater(up[2,0],0.)
        c['center_of_buoyancy']=c['center_of_mass']
        p,v,r,w=self.flight(cfg=c,duration=.1,velocity=np.zeros(3))[-1]
        np.testing.assert_allclose(p,[0,0,-1],atol=1e-12)
        np.testing.assert_allclose(r,np.eye(3),atol=1e-12)

    def test_rotating_flight_timestep_and_current_invariance(self):
        fine=self.flight(dt=.001,duration=.4)[-1]
        coarse=self.flight(dt=.004,duration=.4)[-1]
        for a,b in zip(fine,coarse):np.testing.assert_allclose(a,b,atol=2e-6)
        current=np.array([.1,-.2,0.])
        moved=self.flight(dt=.004,duration=.4,water=current,
                          velocity=np.array([launch_speed(self.cfg['torpedo'],998.2),0,0])+current)[-1]
        np.testing.assert_allclose(moved[0],coarse[0]+current*.4,atol=1e-10)
        np.testing.assert_allclose(moved[1],coarse[1]+current,atol=1e-10)
        np.testing.assert_allclose(moved[2],coarse[2],atol=1e-10)
        # Initial fin roll and world heading must not change the dive behavior.
        r=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        turned=self.flight(dt=.004,duration=.4,orientation=r)[-1]
        np.testing.assert_allclose(turned[0],[-coarse[0][1],coarse[0][0],coarse[0][2]],atol=1e-10)
        self.assertAlmostEqual(turned[2][2,0],coarse[2][2,0])
    def test_timestep_refinement(self):
        c=self.cfg['torpedo'];p=np.array([0.,0.,-1.]);v=np.array([2.5,0,0]);axis=np.array([1.,0,0])
        a,b=p.copy(),v.copy();x,y=p.copy(),v.copy()
        for _ in range(100):
            a,b=advance(a,b,axis,c,np.zeros(3),998.2,.004)
            for _ in range(2):x,y=advance(x,y,axis,c,np.zeros(3),998.2,.002)
        np.testing.assert_allclose(a,x,atol=1e-7);np.testing.assert_allclose(b,y,atol=1e-7)


if __name__=='__main__':unittest.main()
