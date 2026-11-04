#!/usr/bin/env python3
"""
shelf_model.py
==============
Mo hinh ke 4 o (A1/A2/B1/B2) - logic THUAN, khong phu thuoc ROS.

- Doc tham so tu config/shelf.yaml (front_distance_m, col_pitch_m, 4 o,
  anh xa cam bien, thu tu uu tien).
- Bien doi pose D_1 (khung map) -> khung `shelf_front` va khung cua tung
  o, theo nguyen ly: +X cua D_1 (yaw) chi vao ke; o cot nam x = 0 trong
  khung o (doc theo +X), chi lech ngang y = +/-(col_sign * col_pitch_m)
  do cot; z = 0 cho tat ca.
- Trang thai o (full / empty / unknown) tu /shelf/occupancy (Int8MultiArray
  [4], -1 = mat lien lac). Du lieu cua (stale) hay -1 -> 'unknown',
  KHONG bao giu coi la 'empty'.
- Chon o dat lay: chi o loai dung + empty + theo thu tu uu tien; khong
  co o hop le -> tra ly do ro, KHONG fallback.

Chi dung trong amr_mission_manager (rack_tf_publisher, mission_supervisor,
GUI). Chay test:
    python3 src/amr_mission_manager/test/test_shelf_model.py
"""

import math
import os


class ShelfError(ValueError):
    """Loi du lieu cua config/shelf.yaml hoac goi API sai."""


# Trang thai o
UNKNOWN = 'unknown'
EMPTY = 'empty'
FULL = 'full'

# Ten frame ke trong khung map
SHELF_FRONT = 'shelf_front'
SLOT_A = 'slot_A'        # cot 1 (A1 duoi + A2 tren, chung khung)
SLOT_B1 = 'slot_B1'      # cot 2
SLOT_B2 = 'slot_B2'      # cot 3


class Slot:
    """1 o ke: id, cot, hang, loai hang, chi so cam bien, khung o TF."""

    __slots__ = ('id', 'col', 'row', 'product_type',
                 'sensor_index', 'sensor_confirmed', 'tf_frame')

    def __init__(self, raw: dict):
        self.id = str(raw['id'])
        self.col = int(raw['col'])
        self.row = int(raw['row'])
        self.product_type = str(raw['product_type'])
        self.sensor_index = int(raw['sensor_index'])
        self.sensor_confirmed = bool(raw.get('sensor_confirmed', False))
        self.tf_frame = str(raw['tf_frame'])

    def __repr__(self):
        return (f"Slot({self.id} col={self.col} row={self.row} "
                f"type={self.product_type} sensor={self.sensor_index}"
                f"{'(xác nhận)' if self.sensor_confirmed else '(TẠM)'} "
                f"frame={self.tf_frame})")


class ShelfModel:

    def __init__(self, cfg: dict):
        """`cfg` = dict da parse tu shelf.yaml (khong phai duong dan)."""
        self.anchor_node = str(cfg['anchor_node'])
        self.front_distance_m = float(cfg['front_distance_m'])
        self.col_pitch_m = float(cfg['col_pitch_m'])
        self.col_sign = float(cfg.get('col_sign', 1.0))
        self.priority = {str(k): [str(v) for v in lst]
                         for k, lst in (cfg.get('priority') or {}).items()}
        self.slots = [Slot(s) for s in cfg['slots']]
        self._slot_by_id = {s.id: s for s in self.slots}
        self._validate()

    # ------------------------------------------------------------------
    # Nạp config
    # ------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, path: str) -> 'ShelfModel':
        import yaml
        if not os.path.exists(path):
            raise ShelfError(f'Khong tim thay config: {path}')
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        return cls(cfg)

    def _validate(self):
        ids = [s.id for s in self.slots]
        if len(ids) != len(set(ids)):
            raise ShelfError(f'Trùng id o: {ids}')
        sensors = [s.sensor_index for s in self.slots]
        if len(sensors) != len(set(sensors)) or any(s < 0 or s > 3 for s in sensors):
            raise ShelfError(f'sensor_index phai khaC nhau va trong [0,3]: {sensors}')
        for t, prio in self.priority.items():
            known = [i for i in prio if i in self._slot_by_id]
            if len(known) != len(prio):
                raise ShelfError(f"priority[{t}] co id khong ton tai: {prio}")
            for i in known:
                if self._slot_by_id[i].product_type != t:
                    raise ShelfError(f"priority[{t}] chan o loai sai: {i} "
                                     f"(loai={self._slot_by_id[i].product_type})")

    @property
    def frames(self):
        """Danh sach frame o ke RIENG LEU (A1+A2 chung mot khung)."""
        seen = []
        for s in self.slots:
            if s.tf_frame not in seen:
                seen.append(s.tf_frame)
        return seen

    def provisional_sensor_warnings(self):
        """Danh sach canh bao (cho log khoi dong) cua chi so cam bien TAM."""
        return [f"o {s.id} (cam bien chi so {s.sensor_index}, GPIO "
                f"{[23, 22, 21, 19][s.sensor_index]}): TẠM, chua xac nhan - "
                f"che tung cam bien de xac nhan"
                for s in self.slots if not s.sensor_confirmed]

    # ------------------------------------------------------------------
    # Hinh hoc: D_1 -> khung ke (khung map)
    # ------------------------------------------------------------------
    def _slot_local_y(self, slot: Slot) -> float:
        """Toa do y local (khoang `shelf_front` -> tam o) theo cot."""
        if slot.col == 1:
            return self.col_sign * self.col_pitch_m    # ben TRAI (+Y khi sign>0)
        if slot.col == 2:
            return 0.0                                  # tam (cot 2)
        if slot.col == 3:
            return -self.col_sign * self.col_pitch_m     # ben PHAI
        raise ShelfError(f'Cot khong hop le: {slot.col} (o {slot.id})')

    def map_poses(self, d1_x: float, d1_y: float, d1_yaw: float) -> dict:
        """Bien doi pose D_1 (khung map) -> (x, y, yaw) trong map cua
        `shelf_front` va tung khung o RIENG LEU (A1/A2 chung khung ->
        1 nhan trong ket qua). Ket qua khung o KHONG phu thuoc thu tu
        tren/duoi: z = 0 cho tat ca, A1 va A2 TRÙNG toa do (ch y du).

        Quy uoc ROS: nhan theo +X (d1_yaw), ben TRAI = +Y.
        """
        c, s = math.cos(d1_yaw), math.sin(d1_yaw)
        # D1 local -> map: p_map = d1 + R(yaw) * p_local
        fx = d1_x + c * self.front_distance_m
        fy = d1_y + s * self.front_distance_m
        poses = {SHELF_FRONT: (fx, fy, d1_yaw)}
        for s_ in self.slots:
            if s_.tf_frame in poses:
                continue
            ly = self._slot_local_y(s_)
            poses[s_.tf_frame] = (fx - s * ly, fy + c * ly, d1_yaw)
        return poses

    # ------------------------------------------------------------------
    # Trang thai o tu /shelf/occupancy
    # ------------------------------------------------------------------
    def slot_tf_frame(self, slot_id: str):
        """Ten khung o TF (TF chung neu o chung khung) cua 1 o, hoac None."""
        s_ = self._slot_by_id.get(slot_id)
        return s_.tf_frame if s_ else None

    def slot_states(self, data, fresh: bool = True) -> dict:
        """`data` = danh sach 4 gia tri tu /shelf/occupancy (Int8MultiArray).

        - `fresh=False` (du lieu cua / mat lien lac) -> tat ca 'unknown'.
        - `data` co -1 (hay < 0) tai vi tri nao -> o doi ung 'unknown'.
        - 0 -> 'empty', > 0 -> 'full'.
        - KHI NAO CUNG khong bao giu tra 'empty' tu du lieu khac that.
        """
        states = {}
        for s_ in self.slots:
            v = None
            if fresh and data is not None and len(data) > s_.sensor_index:
                v = data[s_.sensor_index]
            if v is None or v < 0:
                states[s_.id] = UNKNOWN
            elif v > 0:
                states[s_.id] = FULL
            else:
                states[s_.id] = EMPTY
        return states

    # ------------------------------------------------------------------
    # Chon o
    # ------------------------------------------------------------------
    def _eligible(self, cargo_type: str):
        prio = self.priority.get(cargo_type, [])
        out = []
        for sid in prio:
            s_ = self._slot_by_id.get(sid)
            if s_ is not None and s_.product_type == cargo_type:
                out.append(s_)
        return out

    def select_drop_slot(self, cargo_type, states: dict):
        """Chon o DATTONG: loai hang dung + o 'empty', theo thu tu uu tien.

        Tra (id_o | None, nguyen_nhan). Khong co o hop le -> (None, ly do),
        KHONG co fallback o du phong. `cargo_type` la 'A' hay 'B';
        None/khac -> khong cho dat (chưa chon loai hang).
        """
        if cargo_type not in self.priority:
            return None, (f"loai hang chua duoc chon (cargo_type={cargo_type!r}); "
                          f"khong cho dat hang - chon A hoac B trong GUI")
        skips = []
        for s_ in self._eligible(cargo_type):
            st = states.get(s_.id, UNKNOWN)
            if st == EMPTY:
                return s_.id, ''
            skips.append(f"{s_.id}={st}")
        return None, ('khong co o hop le de dat (chi dat vao o "trống" cua '
                      f'dung loai; "unknown"/"full" deu kho dat): {", ".join(skips)}')

    def select_pick_slot(self, cargo_type, states: dict):
        """Chon o LẤY (chi dung khi co logic lay tu ke): chi lay tu o
        "co hang" (full), loai dung, theo thu tu uu tien.

        Tra (id_o | None, nguyen_nhan)."""
        if cargo_type not in self.priority:
            return None, f'loai hang chua duoc chon (cargo_type={cargo_type!r})'
        skips = []
        for s_ in self._eligible(cargo_type):
            st = states.get(s_.id, UNKNOWN)
            if st == FULL:
                return s_.id, ''
            skips.append(f"{s_.id}={st}")
        return None, (f"khong co o co hang de lay (chi lay tu o "
                      f'"co hàng"; "empty"/"unknown" deu kho lay): {", ".join(skips)}')
