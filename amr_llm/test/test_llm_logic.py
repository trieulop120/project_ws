#!/usr/bin/env python3
"""
test_llm_logic.py - Test don vi logic thuan cua llm_logic (KHONG can ROS).

Chay (tu goi ~/project_ws):
    python3 src/amr_llm/test/test_llm_logic.py

Nam nhom test:
1. extract_json: JSON thuan, markdown fence.
2. mission_from_to: tach ten mieu (return_home -> None/home).
3. validate_plan: hanh dong la, tham so la, gia tri la, vi tri xuat phat,
   do dai, chan do cao chua do (rack_*).
4. is_stop: nhan lenh dung ngan, khong nhan lenh dai.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'amr_llm'))

import llm_logic as L          # noqa: E402


def check(name, cond, extra=''):
    cond = bool(cond)
    print(f'  [{"OK  " if cond else "FAIL"}] {name}{(" - " + extra) if extra else ""}')
    return cond


def raises(fn, *args, **kwargs):
    """Tra (True, msg) neu fn nem ValueError, (False, '') neu khong."""
    try:
        fn(*args, **kwargs)
    except ValueError as e:
        return True, str(e)
    except Exception as e:      # noqa: BLE001
        return False, f'khong phai ValueError: {type(e).__name__}: {e}'
    return False, 'khong nem ValueError'


# ======================================================================
# 1. extract_json
# ======================================================================
def test_extract_json():
    print('\n[1] extract_json')
    ok = True
    ok &= check('JSON thuan', L.extract_json('{"a":1}') == {'a': 1})
    ok &= check('JSON thuan + khoang trang',
                L.extract_json('  {"a":1}\n') == {'a': 1})
    ok &= check('markdown fence ```json',
                L.extract_json('```json\n{"a":1}\n```') == {'a': 1})
    ok &= check('markdown fence ``` (khong co json)',
                L.extract_json('```\n{"a":1}\n```') == {'a': 1})
    return ok


# ======================================================================
# 2. mission_from_to
# ======================================================================
def test_mission_from_to():
    print('\n[2] mission_from_to')
    ok = True
    ok &= check('return_home -> (None, home)',
                L.mission_from_to('return_home') == (None, 'home'))
    ok &= check('p1_to_d1 -> (p1, d1)',
                L.mission_from_to('p1_to_d1') == ('p1', 'd1'))
    ok &= check('home_to_p2 -> (home, p2)',
                L.mission_from_to('home_to_p2') == ('home', 'p2'))
    ok &= check('d1_to_p2 -> (d1, p2)',
                L.mission_from_to('d1_to_p2') == ('d1', 'p2'))
    ok &= check('ten ngoai danh -> ValueError',
                raises(lambda: L.mission_from_to('bay_to_bay'))[0])
    return ok


# ======================================================================
# 3. validate_plan
# ======================================================================
VALID_PLAN = {
    "say": "ok",
    "clarify": None,
    "steps": [
        {"action": "lift_to", "args": {"level": "low"}},
        {"action": "go_mission", "args": {"name": "home_to_p1"}},
        {"action": "pick_pallet", "args": {}},
        {"action": "lift_to", "args": {"level": "carry"}},
        {"action": "go_mission", "args": {"name": "p1_to_d1"}},
        {"action": "drop_pallet", "args": {}},
    ],
}


def test_validate_plan():
    print('\n[3] validate_plan')
    ok = True

    # 3.1 plan hop le
    steps = L.validate_plan(VALID_PLAN, 'home')
    ok &= check('plan hop le -> 6 buoc', len(steps) == 6, f'got {len(steps)}')
    ok &= check('buoc lam sach giu dung args',
                steps[1] == {"action": "go_mission",
                             "args": {"name": "home_to_p1"}})
    ok &= check('drop_pallet la hanh dong hop le',
                any(s['action'] == 'drop_pallet' for s in steps))

    # 3.2 steps rong -> danh sach rong (khong loi)
    ok &= check('steps rong -> []', L.validate_plan({'steps': []}, 'home') == [])
    ok &= check('khong co key steps -> []',
                L.validate_plan({}, 'home') == [])

    # 3.3 hanh dong la
    bad, msg = raises(L.validate_plan,
                      {'steps': [{"action": "fly", "args": {}}]}, 'home')
    ok &= check('hanh dong "fly" bi chan', bad, msg)

    # 3.4 tham so la
    bad, msg = raises(L.validate_plan,
                      {'steps': [{"action": "lift_to",
                                  "args": {"level": "low", "foo": 1}}]}, 'home')
    ok &= check('tham so la (foo) bi chan', bad, msg)

    # 3.5 gia tri khong hop le
    bad, msg = raises(L.validate_plan,
                      {'steps': [{"action": "lift_to",
                                  "args": {"level": "tang 9"}}]}, 'home')
    ok &= check('level "tang 9" bi chan', bad, msg)

    # 3.6 tham so bat buoc thieu
    bad, msg = raises(L.validate_plan,
                      {'steps': [{"action": "go_mission", "args": {}}]}, 'home')
    ok &= check('go_mission thieu "name" bi chan', bad, msg)

    # 3.7 xuat phat sai
    bad, msg = raises(L.validate_plan,
                      {'steps': [{"action": "go_mission",
                                  "args": {"name": "p1_to_d1"}}]}, 'home')
    ok &= check('go_mission sai xuat phat bi chan', bad,
                f"thong bao co 'dang o': {'dang ở' in msg}")
    ok &= check('thong bao sai xuat phat noi ro vi tri',
                'home' in msg)

    # 3.8 vuot max_steps
    long_plan = {'steps': [{"action": "lift_to", "args": {"level": "low"}}] * 9}
    bad, msg = raises(L.validate_plan, long_plan, 'home')
    ok &= check('> max_steps bi chan', bad, msg)
    ok &= check('9 buoc -> len chinh xac 8 van OK',
                len(L.validate_plan(
                    {'steps': [{"action": "lift_to", "args": {"level": "low"}}] * 8},
                    'home')) == 8)

    # 3.9 return_home hop le o bat ky vi tri
    ok &= check('return_home o vi tri bat ky',
                L.validate_plan(
                    {'steps': [{"action": "go_mission",
                                "args": {"name": "return_home"}}]}, 'p1')
                == [{"action": "go_mission", "args": {"name": "return_home"}}])

    # 3.10 do cao chua do (rack_*)
    for lvl in L.UNMEASURED_LIFT_LEVELS:
        bad, msg = raises(L.validate_plan,
                          {'steps': [{"action": "lift_to",
                                      "args": {"level": lvl}}]}, 'home')
        ok &= check(f'lift_to {lvl} bi chan khi allow_unmeasured_lift=False',
                    bad, msg)
        ok &= check(f'thong bao {lvl} noi ro "chua do"',
                    'chưa đo' in msg or 'chua do' in msg)
    ok &= check('lift_to rack_2 OK khi allow_unmeasured_lift=True',
                len(L.validate_plan(
                    {'steps': [{"action": "lift_to", "args": {"level": "rack_2"}}]},
                    'home', allow_unmeasured_lift=True)) == 1)

    # 3.11 lift_to carry van binh thuong
    ok &= check('lift_to carry OK',
                len(L.validate_plan(
                    {'steps': [{"action": "lift_to", "args": {"level": "carry"}}]},
                    'home')) == 1)

    # 3.12 drop_pallet co tham so thua
    bad, msg = raises(L.validate_plan,
                      {'steps': [{"action": "drop_pallet",
                                  "args": {"slot": "A1"}}]}, 'home')
    ok &= check('drop_pallet tham so thua bi chan', bad, msg)

    # 3.13 drop_pallet khong tham so
    ok &= check('drop_pallet khong tham so OK',
                len(L.validate_plan(
                    {'steps': [{"action": "drop_pallet", "args": {}}]},
                    'home')) == 1)

    # 3.14 cap do dai (vi tri cap nhat sau go_mission)
    ok &= check('cap nhat vi tri sau go_mission',
                len(L.validate_plan(
                    {'steps': [{"action": "go_mission",
                                "args": {"name": "home_to_p1"}},
                               {"action": "go_mission",
                                "args": {"name": "p1_to_d1"}}]}, 'home')) == 2)

    # 3.15 steps khong phai list
    bad, msg = raises(L.validate_plan, {'steps': 'abc'}, 'home')
    ok &= check('steps khong phai list bi chan', bad, msg)

    return ok


# ======================================================================
# 4. is_stop
# ======================================================================
def test_is_stop():
    print('\n[4] is_stop')
    ok = True
    for w in ('dừng', 'dung lai', 'stop', 'khẩn cấp', 'hủy', 'huỷ', 'thôi'):
        ok &= check(f'"{w}" -> True', L.is_stop(w))
    ok &= check('"STOP" (hoa) -> True', L.is_stop('STOP'))
    ok &= check('"dừng lại" -> True', L.is_stop('dừng lại'))
    ok &= check('"di lay pallet o P1" -> False',
                not L.is_stop('đi lấy pallet ở P1'))
    ok &= check('len > 30 chua "dung" -> False',
                not L.is_stop('xin hãy dừng xe lại ngay bây giờ vì có người'))
    return ok


# ======================================================================
# 5. build_system_prompt / TOOLS
# ======================================================================
def test_prompt():
    print('\n[5] build_system_prompt + TOOLS')
    ok = True
    p = L.build_system_prompt('home')
    ok &= check('co "<<WORLD>>"?', '<<WORLD>>' not in p)
    ok &= check('co vi tri robot', 'home' in p)
    ok &= check('co hanh dong drop_pallet', 'drop_pallet' in p)
    ok &= check('co hanh dong lift_to', 'lift_to' in p)
    ok &= check('go_mission co tap 7 mieu',
                all(m in p for m in L.MISSIONS))
    ok &= check('MISSIONS khop 7 mieu', len(L.MISSIONS) == 7)
    ok &= check('TOOLS co 4 hanh dong', len(L.TOOLS) == 4)
    ok &= check('TOOLS co drop_pallet', 'drop_pallet' in L.TOOLS)
    ok &= check('drop_pallet khong tham so', L.TOOLS['drop_pallet']['params'] == {})
    return ok


def main():
    ok = True
    ok &= test_extract_json()
    ok &= test_mission_from_to()
    ok &= test_validate_plan()
    ok &= test_is_stop()
    ok &= test_prompt()
    print()
    if ok:
        print('KET QUA: TAT CA PASS')
        return 0
    print('KET QUA: CO TEST THAT BAI - xem [FAIL] ben tren')
    return 1


if __name__ == '__main__':
    sys.exit(main())
