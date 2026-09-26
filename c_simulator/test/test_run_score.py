import itertools
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from run_score import RunScore, CourseJudge
from magnet_light_model import pose


class LedgerTest(unittest.TestCase):
    def setUp(self):
        self.s = RunScore()
        self.s.start(10)

    def gate(self, role='repair'):
        self.s.gate(role, -1)

    def hit(self, ident, kind='torpedo', target='fire_large', cls='fire', distance=.5):
        self.s.release_payload(kind, ident, distance)
        self.s.payload_result(kind, ident, 'success', target, cls, target.rsplit('_', 1)[-1])

    def test_manual_timer_freezes_and_reset_clears_everything(self):
        self.gate();self.s.light('one')
        self.assertEqual(self.s.snapshot(70)['elapsed'], 60)
        self.s.stop(75)
        self.s.light('two')
        self.assertEqual(self.s.snapshot(1000)['elapsed'], 65)
        self.assertEqual(self.s.points['lights'], 500)
        self.s.reset()
        self.assertEqual(self.s.snapshot(1000)['total'], 0)
        self.assertFalse(self.s.running)

    def test_gate_prerequisite_including_payloads_fired_before_gate(self):
        self.s.release_payload('torpedo', 0, 1.)
        self.s.light('one');self.s.slalom('slalom_front', -1, True)
        self.gate()
        self.s.payload_result('torpedo', 0, 'success', 'fire_large', 'fire', 'large')
        self.assertEqual(sum(self.s.points.values()), 100)

    def test_sequence_is_additive_and_role_sensitive(self):
        self.gate()
        self.hit(0);self.hit(1, target='fire_small')
        self.assertEqual(self.s.points['torpedoes'], 1200)
        self.assertEqual(self.s.points['sequence'], 1400)
        self.assertEqual(self.s.points['distance'], 800)
        self.hit(2)
        self.assertEqual(self.s.points['torpedoes'], 1200)
        self.s.start(0);self.gate('rescue')
        self.hit(0);self.hit(1, target='fire_small')
        self.assertEqual(self.s.points['torpedoes'], 1200)
        self.assertEqual(self.s.points['sequence'], 0)
        self.assertEqual(self.s.target_class, 'blood')

    def test_sequence_uses_launch_order_and_distance_thresholds(self):
        self.gate()
        self.s.release_payload('torpedo', 0, .3048)
        self.s.release_payload('torpedo', 1, .4572-1e-6)
        self.s.payload_result('torpedo', 1, 'success', 'fire_small', 'fire', 'small')
        self.s.payload_result('torpedo', 0, 'success', 'fire_large', 'fire', 'large')
        self.assertEqual(self.s.points['sequence'], 1400)
        self.assertEqual(self.s.points['distance'], 400)
        self.s.start(0);self.gate()
        self.hit(0, target='fire_small');self.hit(1)
        self.assertEqual(self.s.points['sequence'], 0)

    def test_two_bin_markers_one_correct_bonus_per_bin(self):
        self.gate()
        self.hit(0, 'dropper', 'bin_vinyl2');self.hit(1, 'dropper', 'bin_vinyl2')
        self.assertEqual(self.s.points['bins'], 1100)
        self.s.start(0);self.gate()
        self.hit(0, 'dropper', 'bin_vinyl2');self.hit(1, 'dropper', 'bin_vinyl3')
        self.assertEqual(self.s.points['bins'], 1600)
        self.s.light('one');self.s.light('one');self.s.light('two');self.s.light('three')
        self.assertEqual(self.s.points['lights'], 1000)

    def test_best_slalom_award_replaces_lower_and_does_not_stack(self):
        self.gate()
        self.s.slalom('slalom_front', 1, False)
        self.assertEqual(self.s.points['slalom_front'], 200)
        self.s.slalom('slalom_front', -1, True)
        self.s.slalom('slalom_front', 1, True)
        self.assertEqual(self.s.points['slalom_front'], 600)

    def test_table_awards_stack_but_each_object_only_once(self):
        self.gate()
        for key, basket, role in [('plug', 'warning', 'repair'), ('pill', 'helmet', 'rescue')]:
            for _ in range(2):
                self.s.object_event(key, 'grasped', '', basket, role)
                self.s.surface(held=key)
                self.s.object_event(key, 'released', '', basket, role)
                self.s.object_event(key, 'success', basket, basket, role)
        self.assertEqual(self.s.points['objects_surface'], 800)
        self.assertEqual(self.s.points['objects_drop'], 400)
        self.assertEqual(self.s.points['baskets'], 1400)
        self.assertEqual(len(self.s.basket_contents), 2)
        self.s.surface('hammer_and_wrench')
        self.assertEqual(self.s.points['facing'], 700)
        self.s.basket_turns(1)
        self.assertEqual(self.s.points['basket_count'], 500)
        self.s.basket_turns(2)
        self.assertEqual(self.s.points['basket_count'], 1000)
        self.s.object_event('pill', 'grasped', '', 'helmet', 'rescue')
        self.assertEqual(len(self.s.basket_contents), 1)

    def test_facing_tiers_and_zero_basket_count(self):
        self.gate()
        self.s.surface('sos');self.assertEqual(self.s.points['facing'], 200)
        self.s.surface('compass');self.assertEqual(self.s.points['facing'], 400)
        self.s.basket_turns(1);self.assertEqual(self.s.points['basket_count'], 0)

    def test_correct_sorting_is_independent_of_robot_role(self):
        for robot_role, object_role in itertools.product(('repair', 'rescue'), repeat=2):
            with self.subTest(robot_role=robot_role, object_role=object_role):
                self.s.start(0);self.gate(robot_role)
                correct = 'warning' if object_role == 'repair' else 'helmet'
                wrong = 'helmet' if correct == 'warning' else 'warning'
                self.s.object_event('object', 'grasped', '', correct, object_role)
                self.s.object_event('object', 'released', '', correct, object_role)
                self.s.object_event('object', 'wrong_target', wrong, correct, object_role)
                self.assertEqual(self.s.points['objects_drop'], 200)
                self.assertEqual(self.s.points['baskets'], 500)
                self.assertEqual(self.s.snapshot(1)['total'], 100 + 200 + 500)
                self.s.object_event('object', 'grasped', '', correct, object_role)
                self.s.object_event('object', 'released', '', correct, object_role)
                self.s.object_event('object', 'success', correct, correct, object_role)
                self.s.object_event('object', 'success', correct, correct, object_role)
                self.assertEqual(self.s.points['baskets'], 700)
                self.assertEqual(self.s.snapshot(2)['total'], 100 + 200 + 700)

    def test_restore_total_adds_surface_facing_and_each_objects_awards(self):
        self.gate()
        self.s.surface()
        self.assertEqual(self.s.snapshot(1)['total'], 100 + 800)
        self.s.surface('sos')
        self.assertEqual(self.s.snapshot(2)['total'], 100 + 800 + 200)
        self.s.surface('compass')
        self.assertEqual(self.s.snapshot(3)['total'], 100 + 800 + 400)
        for key, basket, role in [('plug', 'warning', 'repair'), ('pill', 'helmet', 'rescue')]:
            self.s.object_event(key, 'grasped', '', basket, role)
            self.s.surface(held=key)
            self.s.object_event(key, 'released', '', basket, role)
            self.s.object_event(key, 'success', basket, basket, role)
        self.s.surface('hammer_and_wrench')
        self.assertEqual(self.s.snapshot(4)['total'], 100 + 800 + 700 + 2 * (200 + 700 + 400))
        self.s.basket_turns(1)
        self.assertEqual(self.s.snapshot(5)['total'], 100 + 800 + 700 + 2 * (200 + 700 + 400) + 500)
        self.s.basket_turns(2)
        self.assertEqual(self.s.snapshot(6)['total'], 100 + 800 + 700 + 2 * (200 + 700 + 400) + 1000)

    def test_random_role_and_heading_coin_awarded_after_matching_gate(self):
        self.s.start(0, 'rescue', heading_coin=True, role_coin=True)
        self.assertEqual(self.s.intended_role, 'rescue')
        self.assertEqual(sum(self.s.points.values()), 0)
        self.gate('repair');self.assertEqual(self.s.points['gate'], 400)
        self.assertEqual(self.s.role, 'repair')
        self.s.start(0, 'rescue', heading_coin=True, role_coin=True);self.gate('rescue')
        self.assertEqual(self.s.points['gate'], 550)
        self.s.start(0, 'rescue', True);self.gate('rescue')
        self.assertEqual(self.s.points['gate'], 400)

    def test_pinger_disabled_by_default_and_early_switch_invalidates_random(self):
        self.gate();self.hit(0)
        self.assertEqual(self.s.points['pinger_first'], 0)
        self.s.start(0);self.gate()
        self.s.select_pinger('deploy', True);self.s.switch_pinger()
        self.s.surface()
        self.assertEqual(self.s.points['pinger_first'], 0)

    def test_pinger_only_new_points_at_selected_tasks_and_distinct_second(self):
        self.gate();self.s.select_pinger('deploy', True)
        self.hit(0);self.assertEqual(self.s.points['pinger_first'], 500)
        self.s.switch_pinger();self.s.switch_pinger()
        self.hit(1, target='fire_small')
        self.assertEqual(self.s.points['pinger_second'], 0)
        self.s.switch_pinger();self.s.surface()
        self.assertEqual(self.s.points['pinger_second'], 1500)


class GeometryTest(unittest.TestCase):
    def setUp(self):
        cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/talos_tasks.yaml').read_text())
        self.cfg=dict(cfg['scoring'], surface_z=0., octagon_apothem=1.36)
        self.frames={key:pose([10,0,-1,0,0,0]) for key in ('table','octagon','slalom_front','slalom_middle','slalom_back')}
        self.frames['gate']=pose([0,0,-.75,0,0,0])
        self.frames['gate_repair']=pose([0,-.75,-.25,0,0,0])
        for name, x, y in [('compass',0,1.38),('hammer_and_wrench',0,-1.38),('buoy',-1.38,0),('sos',1.38,0)]:
            self.frames[name]=pose([10+x,y,-.3,0,0,0])
        self.s=RunScore();self.s.start(0)
        self.j=CourseJudge(self.s,self.cfg,self.frames,np.array(list(itertools.product((-.2,.2),repeat=3))))

    def move(self,x,y=-.75,z=-1,yaw=0,dt=.1):
        self.j.update(pose([x,y,z,0,0,yaw]),dt)

    def gate(self):
        self.move(1);self.move(.1);self.move(-.1)
        self.assertFalse(self.s.gate_passed)
        self.move(-.5)

    def test_full_gate_pass_role_and_return(self):
        self.gate()
        self.assertEqual(self.s.role,'repair')
        self.move(1)
        self.assertEqual(self.s.points['home'],300)

    def test_going_around_gate_does_not_count(self):
        self.move(1,2);self.move(-1,2);self.move(-1,0)
        self.assertFalse(self.s.gate_passed)

    def test_gate_crossing_with_sample_exactly_on_plane(self):
        self.move(1);self.move(0);self.move(-1)
        self.assertTrue(self.s.gate_passed)

    def test_crossing_over_bar_then_descending_does_not_count(self):
        self.move(1,z=-.1);self.move(-.1,z=-.1);self.move(-1,z=-1.)
        self.assertFalse(self.s.gate_passed)

    def test_slalom_crossing_depth_and_no_repeat_farming(self):
        self.gate()
        self.move(11);self.move(9)
        self.assertEqual(self.s.points['slalom_front'],600)
        self.move(11);self.move(9)
        self.assertEqual(self.s.points['slalom_front'],600)

    def test_surface_inside_facing_and_breach_freeze(self):
        self.gate()
        self.move(10,0,-1)
        self.move(10,0,-.19,math.pi/2,dt=.6)
        self.move(10,0,-.19,math.pi/2,dt=.6)
        self.assertEqual(self.s.points['surface'],800)
        self.assertEqual(self.s.points['facing'],400)
        self.move(11.5,0,0,dt=.1)
        self.assertFalse(self.s.accepts)
        self.assertTrue(self.s.running)  # timer is manual

    def test_spawn_at_surface_is_not_a_scored_surface(self):
        self.s.gate('repair',-1)
        self.move(10,0,0,dt=2)
        self.assertEqual(self.s.points['surface'],0)

    def test_gate_style_reversal_cancels_and_cap_is_shared(self):
        self.gate()
        for angle in np.linspace(0,math.pi/2,20):self.move(-.5,yaw=angle)
        for angle in np.linspace(math.pi/2,0,20):self.move(-.5,yaw=angle)
        self.j.finish_gate_attempt()
        self.assertEqual(self.s.points['gate'],100)
        self.j.gate_attempt=dict(passed=True,role='repair',side=-1,turns=np.array([4*math.pi,0,4*math.pi]))
        self.j.finish_gate_attempt()
        self.assertEqual(self.s.points['gate'],1700)

    def test_turn_signal_is_measured_at_surface_after_heading_settles(self):
        self.gate()
        self.s.basket_contents={'plug':'warning','pill':'helmet'}
        self.move(10,0,-1)
        self.move(10,0,-.19,dt=.6)
        for angle in np.linspace(0,4*math.pi,121):self.move(10,0,-.19,angle)
        self.assertEqual(self.s.points['basket_count'],0)
        self.move(10,0,-.19,0,dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)

    def test_only_partial_footprint_inside_is_a_breach_not_full_credit(self):
        self.gate()
        self.move(11.3,0,-1)
        self.move(11.3,0,0,dt=1.)
        self.assertEqual(self.s.points['surface'],0)
        self.assertTrue(self.s.ended_reason)

    def test_turn_signal_can_be_completed_underwater(self):
        self.gate();self.s.basket_contents={'plug':'warning'}
        self.move(10,0,-1)
        for angle in np.linspace(0,2*math.pi,61):self.move(10,0,-1,angle)
        self.move(10,0,-1,0,dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)
        self.assertEqual(self.s.points['surface'],0)

    def test_reorienting_before_turn_signal_does_not_offset_count(self):
        self.gate();self.s.basket_contents={'plug':'warning','pill':'helmet'}
        self.move(10,0,-1,dt=.6)
        for angle in np.linspace(0,math.pi/2,16):self.move(10,0,-1,angle)
        self.move(10,0,-1,math.pi/2,dt=2)
        for angle in np.linspace(math.pi/2,math.pi/2+4*math.pi,121):
            # 100 Hz clock ticks can repeat a stale ground-truth pose mid-spin
            self.move(10,0,-1,angle,dt=.02);self.move(10,0,-1,angle,dt=.01)
        self.assertEqual(self.s.points['basket_count'],0)
        self.move(10,0,-1,math.pi/2,dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)

    def test_slow_lead_limited_spin_is_not_split(self):
        self.gate();self.s.basket_contents={'plug':'warning','pill':'helmet','bandage':'helmet'}
        self.move(10,0,-1,dt=1.2)
        angle=0.
        while angle<6*math.pi:
            # ~11 deg/s average with brief hesitations, like the sim's spin
            for _ in range(3):self.move(10,0,-1,angle,dt=.1)
            angle=min(6*math.pi,angle+math.radians(4))
        self.assertEqual(self.s.points['basket_count'],0)
        self.move(10,0,-1,angle,dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)

    def spin(self,start,stop,step=math.radians(4)):
        for angle in np.arange(start,stop,np.sign(stop-start)*step):self.move(10,0,-1,angle)
        self.move(10,0,-1,stop)

    def test_continuous_turns_ignore_reorientation_and_allow_shortfall(self):
        self.gate();self.s.basket_contents={'plug':'warning','pill':'helmet','bandage':'helmet'}
        self.move(10,0,-1,dt=1.2)
        self.spin(0,-math.radians(150))                       # face the sign the other way
        self.spin(-math.radians(150),-math.radians(150)+6*math.pi-math.radians(25))
        self.assertEqual(self.s.points['basket_count'],0)
        self.move(10,0,-1,-math.radians(150)+6*math.pi-math.radians(25),dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)   # 25 deg short still counts
        self.assertTrue(any('reversed' in line for line in self.j.turn_log))
        self.assertTrue(any('basket_count 0 -> 1000' in line for line in self.j.turn_log))

    def test_stops_keep_the_count_going(self):
        self.gate();self.s.basket_contents={'plug':'warning','pill':'helmet','bandage':'helmet'}
        self.move(10,0,-1,dt=1.2)
        self.spin(0,2*math.pi)
        self.move(10,0,-1,2*math.pi,dt=2)
        self.spin(2*math.pi,6*math.pi)
        self.move(10,0,-1,6*math.pi,dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)

    def test_reversal_past_threshold_restarts_the_count(self):
        self.gate();self.s.basket_contents={'plug':'warning','pill':'helmet','bandage':'helmet'}
        self.move(10,0,-1,dt=1.2)
        self.spin(0,2*math.pi)
        self.spin(2*math.pi,2*math.pi-math.radians(20))
        self.spin(2*math.pi-math.radians(20),6*math.pi-math.radians(20))
        self.move(10,0,-1,6*math.pi-math.radians(20),dt=1.2)
        self.assertEqual(self.s.points['basket_count'],500)    # 2 turns after the reversal
        self.assertTrue(any('count restarts' in line for line in self.j.turn_log))

    def test_small_backoff_does_not_restart_the_count(self):
        self.gate();self.s.basket_contents={'plug':'warning','pill':'helmet','bandage':'helmet'}
        self.move(10,0,-1,dt=1.2)
        self.spin(0,2*math.pi)
        self.spin(2*math.pi,2*math.pi-math.radians(10))
        self.spin(2*math.pi-math.radians(10),6*math.pi)
        self.move(10,0,-1,6*math.pi,dt=1.2)
        self.assertEqual(self.s.points['basket_count'],1000)

    def test_turns_are_judged_on_leaving_table_area(self):
        self.gate();self.s.basket_contents={'plug':'warning'}
        self.move(10,0,-1)
        self.spin(0,2*math.pi)
        for x in np.arange(10,14,.2):self.move(x,0,-1,2*math.pi,dt=.02)  # exits before a 1 s stop
        self.assertEqual(self.s.points['basket_count'],1000)
        self.assertTrue(any('left table area' in line for line in self.j.turn_log))

    def test_teleport_cannot_sweep_through_slalom_for_points(self):
        self.gate()
        self.move(14);self.move(6)
        self.assertEqual(self.s.points['slalom_front'],0)


if __name__=='__main__':unittest.main()
