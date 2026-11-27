import math
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from amr_mission_manager.shelf_model import ShelfModel
from amr_web.shelf_visual import rack_from_route
from amr_web.warehouse_view import WarehouseView


class WarehouseViewTest(unittest.TestCase):
    @patch('amr_web.warehouse_view.time.monotonic')
    def test_floor_pickup_preview_lock_approach_and_release(self, clock):
        clock.return_value = 10
        view = WarehouseView()
        scope = dict(session_id='test', operation_id=7, mode='pick')
        def mission(state, **fields):
            view.mission('site', {**scope, 'state':state, **fields}, None)
        def latest():
            return view.snapshot('site','map')
        face = dict(x=1.5,y=-.4,z=.025,yaw=.5)
        mission('DETECTING', cargo=False)
        view.camera_cargo('site', {'class':'medium', 'height_m':.244})
        self.assertIsNone(latest()['target'])
        clock.return_value += .12
        view.pose('site',face,stamp=101)
        target = latest()['target']
        self.assertEqual(target['height_m'], .244)
        self.assertFalse(target['locked'])
        self.assertIsNone(target['slot'])
        clock.return_value += .1
        moved = {**face, 'x':1.52}
        view.pose('site',moved,stamp=102)
        view.pose('site',face,stamp=101)
        view.camera_cargo('other', {'class':'tall', 'height_m':.9})
        view.camera_cargo('site', {'class':'medium', 'height_m':.247})
        self.assertEqual(latest()['target']['pose'], moved)
        self.assertEqual(latest()['target']['height_m'], .247)
        mission('READY', cargo=False,cargo_h_cm=24.9)
        view.pose('site',{**face,'x':1.6},stamp=103)
        view.camera_cargo('site', {'class':'tall', 'height_m':.8})
        self.assertEqual(latest()['target']['id'], target['id'])
        self.assertEqual(latest()['target']['pose'], moved)
        self.assertEqual(latest()['target']['height_m'], .249)
        self.assertTrue(latest()['target']['locked'])
        scope['operation_id'] = 8  # GO advances the operation after READY.
        mission('ALIGNING', cargo=False)
        mission('LIFTING', cargo=False,us_pick_confirmed=True)
        self.assertIsNone(latest()['carried'])
        self.assertEqual(latest()['target']['id'], target['id'])
        mission('BACKING', cargo=True,us_pick_confirmed=True)
        self.assertIsNone(latest()['target'])
        self.assertEqual(latest()['carried']['id'], target['id'])
        self.assertEqual(latest()['carried']['height_m'], .249)
        mission('CARRYING', cargo=True)
        mission('DONE', cargo=False)
        self.assertIsNone(latest()['carried'])
        self.assertIsNone(latest()['target'])
        self.assertEqual(latest()['release']['id'], target['id'])

    @patch('amr_web.warehouse_view.time.monotonic')
    def test_untimed_height_and_missed_detection_are_scoped(self, clock):
        clock.return_value = 10
        view = WarehouseView()
        status = dict(session_id='test', operation_id=1,mode='pick',state='DETECTING')
        view.mission('site',status,None)
        view.camera_cargo('site',{'class':'tall', 'height_m':.4})
        clock.return_value = 10.5
        face = dict(x=1,y=2,z=.025,yaw=0)
        view.pose('site',face,stamp=1)
        self.assertIsNone(view.snapshot('site','map')['target'].get('height_m'))
        clock.return_value = 14
        view.mission('site',status,None)
        self.assertIsNone(view.snapshot('site','map')['target'])
        view.camera_cargo('site',{'class':'medium', 'height_m':.2})
        view.mission('site',{**status,'operation_id':2},None)
        view.pose('site',face,stamp=2)
        self.assertIsNone(view.snapshot('site','map')['target'].get('height_m'))

    def test_confirmed_pick_and_drop_survive_failure_and_wait_for_inventory(self):
        view = WarehouseView()
        def mission(state, **fields):
            return dict(session_id='test', operation_id=1, mode='pick', state=state, **fields)
        def snapshot():
            return view.snapshot('site', 'map')
        view.mission('site', mission('DETECTING', shelf={'fresh': True, 'states': {'A1': 'full'}}), None)
        self.assertIsNone(snapshot()['target'])
        retrieval = {'state': 'DETECTING', 'run_id': 'retrieval1', 'slot': 'A1'}
        view.pose('site', dict(x=1, y=2, z=.2, yaw=0), retrieval)
        self.assertEqual(snapshot()['target']['id'], 'shelf_A1')
        view.mission('site', mission('FAILED', cargo=False), retrieval)
        self.assertIsNotNone(snapshot()['target'])
        self.assertIsNone(snapshot()['carried'])
        view.mission('site', mission('BACKING', cargo=True, us_pick_confirmed=True), retrieval)
        self.assertEqual(snapshot()['carried']['id'], 'shelf_A1')
        view.mission('site', mission('FAILED', cargo=None), retrieval)
        self.assertEqual(snapshot()['carried']['id'], 'shelf_A1')
        view.mission('site', {'state': 'DONE', 'mode': 'drop', 'cargo': False,
                             'drop': {'slot': 'B1'}, 'shelf': {'fresh': True, 'states': {'B1': 'empty'}}}, None)
        self.assertIsNone(snapshot()['carried'])
        self.assertEqual(snapshot()['slots']['B1'], 'empty')
        self.assertEqual(snapshot()['drops']['B1']['id'], 'shelf_A1')
        view.mission('site', {'shelf': {'fresh': True, 'states': {'B1': 'full'}}}, None)
        self.assertEqual(snapshot()['slots']['B1'], 'full')
        self.assertEqual(snapshot()['drops']['B1']['id'], 'shelf_A1')
        self.assertEqual(view.snapshot('different site', 'map')['slots'], {})

    def test_drop_confirmation_precedes_backoff_and_fences_old_cargo(self):
        view = WarehouseView()
        def send(op, phase, mode='drop', cargo=True, occupancy='empty', **fields):
            view.mission('site', dict(session_id='node', operation_id=op, state=phase,
                                      mode=mode, cargo=cargo, drop={'slot':'A2'},
                                      shelf={'fresh':True,'states':{'A2':occupancy}}, **fields), None)
            return view.snapshot('site', 'map')
        load = send(7, 'CARRYING', 'pick')['carried']
        before = send(8, 'LIFTING', occupancy='full')
        self.assertEqual((before['pending_slot'], before['carried']), ('A2', load))
        self.assertEqual(before['drops'], {})
        placed = send(8, 'LIFTING', visual_release={'operation_id':8, 'destination':'rack_slot'})
        self.assertEqual(placed['drops']['A2']['id'], load['id'])
        for snapshot in (send(8, 'BACKING', occupancy='full'), send(8, 'LOWERING'),
                         send(7, 'CARRYING', 'pick'), send(8, 'CARRYING', 'pick'),
                         send(8, 'DONE', cargo=False, occupancy='full')):
            self.assertIsNone(snapshot['carried'])
            self.assertEqual(snapshot['drops']['A2']['id'], load['id'])

    @patch('amr_web.warehouse_view.time.monotonic')
    def test_d2_release_confirmation_age_survives_repeats(self, clock):
        view = WarehouseView()
        retrieval = dict(state='PICKING', run_id='r', slot='B2')
        def send(op, phase, cargo, **fields):
            view.mission('site', dict(session_id='node', operation_id=op, state=phase,
                                      mode='pick', cargo=cargo, **fields), retrieval)
            return view.snapshot('site', 'map')
        clock.return_value = 100
        load = send(1, 'CARRYING', True)['carried']
        retrieval['state'] = 'PLACING_AT_D2'
        for phase, cargo in [('LOWERING',True), ('FAILED',False)]:
            snap = send(2, phase, cargo)
            self.assertEqual(snap['carried'], load)
            self.assertIsNone(snap['release'])
        # Coordinator may already have advanced before the bridge gets DONE.
        retrieval['state'] = 'RETURNING_HOME'
        snap = send(2, 'DONE', False, visual_release={'operation_id':2, 'destination':'D_2', 'age_s':.25})
        self.assertEqual(snap['release_age_s'], .25)
        clock.return_value += 1
        self.assertEqual(send(2, 'DONE', False)['release_age_s'], 1.25)
        self.assertIsNone(send(1, 'CARRYING', True)['carried'])
        retrieval['run_id'] = 'new'
        self.assertIsNone(send(3, 'IDLE', False)['release'])

    def test_detector_target_does_not_become_carried_without_confirmation(self):
        view = WarehouseView()
        locked_pose = dict(x=1, y=2, z=.2, yaw=0)
        view.mission('map a', {'state': 'DETECTING', 'mode': 'pick',
                               'session_id': 's', 'operation_id': 3}, None)
        view.pose('map a', locked_pose)
        view.mission('map a', {'state': 'READY', 'mode': 'pick', 'cargo': False, 'cargo_h_cm': 21,
                               'session_id': 's', 'operation_id': 3}, None)
        snapshot = view.snapshot('map a', 'map')
        self.assertEqual(snapshot['target']['pose'], locked_pose)
        self.assertEqual(snapshot['target']['height_m'], .21)
        self.assertIsNone(snapshot['carried'])
        view.mission('map a', {'state': 'BACKING', 'mode': 'pick', 'cargo': True,
                               'session_id': 's', 'operation_id': 3}, None)
        self.assertIsNone(view.snapshot('map a', 'map')['carried'])

    def test_retrieval_release_does_not_invent_a_destination_pose(self):
        view = WarehouseView()
        retrieval = {'state': 'PICKING', 'run_id': 'r1', 'slot': 'A2'}
        view.mission('map a', {'state': 'DETECTING'}, retrieval)
        view.pose('map a', dict(x=0, y=0, z=.55, yaw=0), retrieval)
        view.mission('map a', {'state': 'CARRYING', 'mode': 'pick', 'cargo': True,
                               'session_id': 's', 'operation_id': 2}, retrieval)
        view.mission('map a', {'state': 'DONE', 'mode': 'pick', 'cargo': False},
                     {**retrieval, 'state': 'PLACING_AT_D2'})
        snapshot = view.snapshot('map a', 'map')
        self.assertIsNone(snapshot['carried'])
        self.assertEqual(snapshot['drops'], {})
        self.assertIsNone(snapshot['target'])

    def test_source_and_height_are_preserved_through_pick_release_and_refill(self):
        view = WarehouseView()
        def update(state, cargo, op=1, mode='pick', retrieval=None, **extra):
            view.mission('site', dict(state=state,cargo=cargo,mode=mode,session_id='s',
                                     operation_id=op,**extra), retrieval)
        def shelf(**states):
            return dict(fresh=True, states=states)
        snap = lambda: view.snapshot('site', 'map')
        update('CARRYING', True, cargo_h_cm=24.9)
        update('DONE', False, op=2, mode='drop', drop={'slot':'A2'}, shelf=shelf(A2='full'))
        load = snap()['drops']['A2']
        self.assertEqual(load['height_m'], .249)
        retrieval = dict(state='PICKING',run_id='ret',slot='A2')
        view.pose('site', dict(x=1,y=2,z=.61,yaw=0), retrieval)
        self.assertEqual(snap()['target']['id'], load['id'])
        update('CARRYING', True, op=3,retrieval=retrieval,shelf=shelf(A2='full'))
        self.assertEqual(snap()['carried']['height_m'], .249)
        update('STOPPED', None,op=3,retrieval=retrieval,shelf=shelf(A2='full'))
        self.assertEqual(snap()['carried']['id'], load['id'])
        update('BACKING',False,op=4,retrieval={**retrieval,'state':'PLACING_AT_D2'},shelf=shelf(A2='full'))
        self.assertIsNone(snap()['carried'])
        self.assertEqual(snap()['release'], dict(id=load['id'], destination='D_2', pose=None, run='ret'))
        self.assertIn('A2', snap()['departed'])
        update('IDLE',False,op=4,shelf=shelf(A2='empty'))
        update('IDLE',False,op=4,shelf=shelf(A2='full'))
        self.assertNotIn('A2', snap()['departed'])
        self.assertNotIn('A2', snap()['drops'])

    def test_home_after_floor_pick_detaches_without_placing_at_d2(self):
        view = WarehouseView()
        view.mission('site',dict(session_id='s',operation_id=1,state='CARRYING',mode='pick',cargo=True),None)
        view.mission('site',dict(session_id='s',operation_id=1,state='FAILED',mode='pick',cargo=False),None)
        self.assertIsNotNone(view.snapshot('site','map')['carried'])
        view.mission('site',dict(session_id='s',operation_id=1,state='DONE',mode='pick',cargo=False),None)
        snap = view.snapshot('site','map')
        self.assertIsNone(snap['carried'])
        self.assertEqual(snap['release']['destination'],'unlocalized')
        self.assertIsNone(snap['release']['pose'])


class ShelfGeometryTest(unittest.TestCase):
    def test_surveyed_dimensions_from_config_and_rotated_route_anchor(self):
        cfg = Path(__file__).resolve().parents[2] / "amr_mission_manager/config/shelf.yaml"
        model = ShelfModel.from_yaml(str(cfg))
        anchor = PoseStamped()
        anchor.header.frame_id = 'map'
        anchor.pose.position.x = 2.0
        anchor.pose.position.y = -3.0
        anchor.pose.orientation.z = math.sin(math.pi/4)
        anchor.pose.orientation.w = math.cos(math.pi/4)
        route = SimpleNamespace(resolve=lambda name: name, get_node_pose=lambda name: anchor)
        rack = rack_from_route(model, route, 'map')
        self.assertEqual((rack['width'], rack['depth'], rack['height']), (1.875, .35, .9))
        self.assertEqual(len(rack['partitions']), 2)
        self.assertAlmostEqual(rack['pose']['y'], -3 + 1.20 + .35/2)
        slots = {slot['id']: slot['pose'] for slot in rack['slots']}
        self.assertEqual({name: slot['z'] for name, slot in slots.items()},
                         {'A1': .2, 'A2': .55, 'B1': .2, 'B2': .2})
        self.assertAlmostEqual(abs(slots['A1']['x']-slots['B1']['x']), .625)
        self.assertEqual({slot['id']:slot['cargo_height_m'] for slot in rack['slots']},
                         {'A1':.15, 'A2':.15, 'B1':.40, 'B2':.40})
        self.assertIsNone(rack_from_route(model, route, 'other_frame'))


if __name__ == '__main__':
    unittest.main()
