#!/usr/bin/env python3
"""
mission_supervisor.launch.py
============================
Chay mission_supervisor node (MỚI) - điều phối flow "lấy pallet" demo:

    HOME ->(mission_manager) P_1 ->(pallet_mission) detect/insert/lift/back/home

KHÔNG đụng node khác: mission_manager va pallet_mission PHAI DANG CHAY
(pallet_mission.launch.py đã chạy cả hai). Chạy launch này riêng khi demo,
với mission_manager/pallet_mission đang chạy (with_mission_manager:=false).

Vidu:
    ros2 launch amr_mission_manager mission_supervisor.launch.py
    ros2 service call /mission/start std_srvs/srv/Trigger '{}'
    # override mission TRUOC khi start (hot, khong can restart):
    ros2 param set /mission_supervisor pick_mission home_to_p2
    ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def _arg(name, default, desc):
    return DeclareLaunchArgument(name, default_value=default, description=desc)


def generate_launch_description():
    # ================================================================
    # PATHS - chinh sua o day (ten mission + diem den trong route graph)
    # ================================================================
    # Mission cua mission_manager (khop MISSION_BT_MAP):
    #   edge -> BT ComputeRoute, ngat edge -> fallback ComputePathToPose
    PICK_MISSION = 'home_to_p1'      # HOME -> P_1
    PICK_DEST = 'P_1'                 # node graph de soi "da den choi" (TF map->base)

    DROP_MISSION = 'p1_to_d1'        # P_1 -> D_1 (dung khi run_drop:=true + co mode drop)
    DROP_BIN = 'bin_0'               # frame TF cua rack (rack_tf_publisher)

    # Node phuc vu
    MISSION_MANAGER_NODE = 'mission_manager'
    PALLET_MISSION_NODE = 'pallet_mission'

    # Sampling (detector 1-1.5 Hz): gom 6s ~ 7-9 mau; cua so estimate 4s
    PICK_SAMPLE_WAIT_S = 6.0
    # Bo nua cuoi "KHONG ON DINH" (ap len pallet_mission HOT qua SetParameters)
    PM_FINAL_WINDOW_S = '4.0'     # cua so cuoi (s) de chot median
    PM_MIN_SAMPLES = '3'          # ~2.5s gom du mau o 1.2Hz; median 3 mau chan 1 outlier
    PM_MAX_POS_STD_M = '0.05'     # 5cm (nho hơn gan 2 lan 3cm, dep cho 1-1.5Hz)
    PM_MAX_YAW_STD_DEG = '5.0'    # 5deg
    PM_MAX_SAMPLE_AGE_S = '3.0'   # 1 mau 0.83s; du lech 2-3 chu ky
    PM_REQUIRE_STABLE = 'true'    # nua nua cuoi: false = bo luon kiem tra std

    # ==================== Launch arguments =========================
    # W4 (REPORT_rack_esp32_network.md): nguồn trạng thái ô kệ
    BIN_SOURCE = 'none'             # 'none' | 'esp32' (rack_mqtt_bridge)
    RACK_ID = 'R1'
    RACK_STALE_S = '10.0'
    DROP_BIN_FALLBACK = 'bin_0'

    args = [
        _arg('pick_mission', PICK_MISSION,
             'mission cua mission_manager de ve node pick (edge/set-goal)'),
        _arg('pick_dest', PICK_DEST, 'ten node graph (map frame) de soi "da den"'),
        _arg('drop_mission', DROP_MISSION, 'mission ve node drop (run_drop)'),
        _arg('drop_bin', DROP_BIN, 'frame bin trong rack (rack_tf_publisher)'),
        _arg('run_drop', 'false', 'true: sau pick thanh GOTO_D/DROPPING (can mode drop)'),
        _arg('mission_manager_node', MISSION_MANAGER_NODE, 'ten node mission_manager'),
        _arg('pallet_mission_node', PALLET_MISSION_NODE, 'ten node pallet_mission'),
        _arg('pick_sample_wait_s', str(PICK_SAMPLE_WAIT_S),
             'thoi gian gom mau detect (s)'),
        _arg('pm_final_window_s', PM_FINAL_WINDOW_S,
             'ap len pallet_mission: cua so cuoi de chot median (s)'),
        _arg('pm_min_samples', PM_MIN_SAMPLES,
             'ap len pallet_mission: so mau toi thieu (tong + trong cua so)'),
        _arg('pm_max_pos_std_m', PM_MAX_POS_STD_M,
             'ap len pallet_mission: nguong std vi tri (m)'),
        _arg('pm_max_yaw_std_deg', PM_MAX_YAW_STD_DEG,
             'ap len pallet_mission: nguong std yaw (deg)'),
        _arg('pm_max_sample_age_s', PM_MAX_SAMPLE_AGE_S,
             'ap len pallet_mission: mau cuoi co tre namay nua (s)'),
        _arg('pm_require_stable', PM_REQUIRE_STABLE,
             'ap len pallet_mission: true = nhan duong "KHONG ON DINH"'),
        _arg('arrive_tol_m', '0.30', 'ban kinh "da den" node pick (m)'),
        _arg('arrive_settle_s', '1.0', 'thoi gian on dinh khi "da den" (s)'),
        _arg('goto_timeout_s', '120.0', 'gioi thoi gian ve node pick (s)'),
        _arg('pick_retries', '2', 'so lan thu detect lai khi that (lan)'),
        # W4: nguon trang thai o ke
        _arg('bin_source', BIN_SOURCE,
             "'none' (tu chon o / siu am) | 'esp32' (rack_mqtt_bridge)"),
        _arg('rack_id', RACK_ID, 'rack_id noi bat trong /rack/occupancy'),
        _arg('rack_stale_s', RACK_STALE_S, 'khe stal > nguong -> moi o unknown (s)'),
        _arg('drop_bin_fallback', DROP_BIN_FALLBACK,
             'drop_bin=auto ma khong co o tron: dung o nay'),
    ]

    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    def pv(name, typ):
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    supervisor = Node(
        package='amr_mission_manager',
        executable='mission_supervisor',
        name='mission_supervisor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'pick_mission': LaunchConfiguration('pick_mission'),
            'pick_dest': LaunchConfiguration('pick_dest'),
            'drop_mission': LaunchConfiguration('drop_mission'),
            'drop_bin': LaunchConfiguration('drop_bin'),
            'run_drop': pv('run_drop', bool),
            'mission_manager_node': LaunchConfiguration('mission_manager_node'),
            'pallet_mission_node': LaunchConfiguration('pallet_mission_node'),
            'pick_sample_wait_s': pv('pick_sample_wait_s', float),
            'pm_final_window_s': pv('pm_final_window_s', float),
            'pm_min_samples': pv('pm_min_samples', int),
            'pm_max_pos_std_m': pv('pm_max_pos_std_m', float),
            'pm_max_yaw_std_deg': pv('pm_max_yaw_std_deg', float),
            'pm_max_sample_age_s': pv('pm_max_sample_age_s', float),
            'pm_require_stable': pv('pm_require_stable', bool),
            'arrive_tol_m': pv('arrive_tol_m', float),
            'arrive_settle_s': pv('arrive_settle_s', float),
            'goto_timeout_s': pv('goto_timeout_s', float),
            'pick_retries': pv('pick_retries', int),
            # W4: nguon trang thai o ke
            'bin_source': LaunchConfiguration('bin_source'),
            'rack_id': LaunchConfiguration('rack_id'),
            'rack_stale_s': pv('rack_stale_s', float),
            'drop_bin_fallback': LaunchConfiguration('drop_bin_fallback'),
        }],
    )
    return LaunchDescription(args + [supervisor])
