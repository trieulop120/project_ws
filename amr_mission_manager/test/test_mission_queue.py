import sys
from pathlib import Path
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from amr_mission_manager.mission_queue import (
    mission_start_blockers, normalize_mission_queue, task_after_placement,
)


class MissionQueueTests(unittest.TestCase):
    def test_two_medium_remains_one_two_pallet_task(self):
        queue = normalize_mission_queue('', {
            'cargo_class': 'medium', 'quantity': 2, 'drop_slot': 'auto'})
        self.assertEqual(queue, [{'cargo_class': 'medium', 'quantity': 2, 'drop_slot': 'auto'}])
        self.assertEqual(task_after_placement(queue, 0, 1), ('repeat', 0))
        self.assertEqual(task_after_placement(queue, 0, 2), ('complete', 0))

    def test_two_any_pallets_remains_one_two_pallet_task(self):
        queue = normalize_mission_queue('', {
            'cargo_class': 'any', 'quantity': 2, 'drop_slot': 'auto'})
        self.assertEqual(queue[0]['cargo_class'], 'any')
        self.assertEqual(task_after_placement(queue, 0, 1), ('repeat', 0))
        self.assertEqual(task_after_placement(queue, 0, 2), ('complete', 0))

    def test_medium_a2_then_tall_b2_is_ordered_and_returns_home_last(self):
        queue = normalize_mission_queue([
            {'cargo_class': 'medium', 'quantity': 1, 'drop_slot': 'A2'},
            {'cargo_class': 'tall', 'quantity': 1, 'drop_slot': 'B2'},
        ], {})
        self.assertEqual(task_after_placement(queue, 0, 1), ('next', 1))
        self.assertEqual(task_after_placement(queue, 1, 1), ('complete', 1))

    def test_done_can_start_new_run_but_unresolved_service_still_blocks(self):
        self.assertEqual(mission_start_blockers(
            stopping=False, discarding=False, tick_inflight=False,
            unresolved_services=0, manager_busy=True,
            terminal_idle_confirmed=True), [])
        self.assertIn('1 service outcome(s) are still unresolved', mission_start_blockers(
            stopping=False, discarding=False, tick_inflight=False,
            unresolved_services=1, manager_busy=False,
            terminal_idle_confirmed=True))
        self.assertIn('Mission Manager reports an active navigation run', mission_start_blockers(
            stopping=False, discarding=False, tick_inflight=False,
            unresolved_services=0, manager_busy=True,
            terminal_idle_confirmed=False))


if __name__ == '__main__':
    unittest.main()
