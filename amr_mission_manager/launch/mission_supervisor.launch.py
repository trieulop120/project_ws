#!/usr/bin/env python3
"""
mission_supervisor.launch.py
============================
Chay mission_supervisor + rack_tf_publisher (+ GUI nut START/STOP):

    HOME ->(mission_manager) P_1 ->(pallet_mission) detect/insert/lift/back/home

KHÔNG đụng node khác: mission_manager va pallet_mission PHAI DANG CHAY
(pallet_mission.launch.py đã chạy cả hai). Chạy launch này riêng khi demo.

Vidu:
    ros2 launch amr_mission_manager mission_supervisor.launch.py
    ros2 launch amr_mission_manager mission_supervisor.launch.py gui:=false
    # override mission TRUOC khi start (hot, khong can restart):
    ros2 param set /mission_supervisor pick_mission home_to_p2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


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

    DROP_MISSION = 'p1_to_d1'        # P_1 -> D_1
    # Frame TF cua ke (rack_tf_publisher: bin_low / bin_medium / bin_high)
    # hoac 'auto' = tu chon cot tron dau tien theo /shelf/occupancy (ESP32)

    # Node phuc vu
    MISSION_MANAGER_NODE = 'mission_manager'
    PALLET_MISSION_NODE = 'pallet_mission'

    # Shelf occupancy (loi 2 ESP serial -> /shelf/occupancy, esp32_bridge)
    SHELF_TOPIC = '/shelf/occupancy'
    SHELF_STALE_S = '2.0'            # > 2s khong co dat = coi mat ket noi

    # Sampling (detector 1-1.5 Hz): gom 6s ~ 7-9 mau; cua so estimate 4s
    PICK_SAMPLE_WAIT_S = 12.0
    # Bo nua cuoi "KHONG ON DINH" (ap len pallet_mission HOT qua SetParameters)
    PM_FINAL_WINDOW_S = '4.0'     # cua so cuoi (s) de chot median
    PM_MIN_SAMPLES = '3'          # ~2.5s gom du mau o 1.2Hz; median 3 mau chan 1 outlier
    PM_MAX_POS_STD_M = '0.05'     # 5cm (nho hơn gan 2 lan 3cm, dep cho 1-1.5Hz)
    PM_MAX_YAW_STD_DEG = '5.0'    # 5deg
    PM_MAX_SAMPLE_AGE_S = '3.0'   # 1 mau 0.83s; du lech 2-3 chu ky
    PM_REQUIRE_STABLE = 'true'    # nua nua cuoi: false = bo luon kiem tra std

    # ================================================================
    # Rack TF (rack_tf_publisher) - 3 cot ke, neo theo anchor trong graph
    # ================================================================
    WITH_RACK_TF = 'true'
    RACK_GRAPH_YAML = 'D203B Route.yaml' # graph chay that (D_1 phai co trong file nay); doi graph
                                          #   -> doi file nay, ke phat TF khi co D_1)
    # Hinh hoc ke (anchor D_1, front_distance_m, col_pitch_m, col_sign) nam trong config/shelf.yaml
    RACK_PUBLISH_ONCE = 'false'          # true: kiem tra 1 lan roi thoat (ke fix)
    RACK_CHECK_INTERVAL_S = '5.0'        # chu ki quan sat khi publish_once=false

    # GUI nut START / STOP
    WITH_GUI = 'true'

    # ==================== Launch arguments =========================
    args = [
        _arg('pick_mission', PICK_MISSION,
             'mission cua mission_manager de ve node pick (edge/set-goal)'),
        _arg('pick_dest', PICK_DEST, 'ten node graph (map frame) de soi "da den"'),
        _arg('drop_mission', DROP_MISSION, 'mission tu P_1/P_2 toi D_1'),
        _arg('run_mode', 'single', 'single: 1 chu ky + HOME | auto: lap den khi het pallet/slot'),
        _arg('mission_manager_node', MISSION_MANAGER_NODE, 'ten node mission_manager'),
        _arg('pallet_mission_node', PALLET_MISSION_NODE, 'ten node pallet_mission'),
        _arg('shelf_topic', SHELF_TOPIC, 'topic 4 bit o ke tu esp32_bridge (Int8MultiArray)'),
        _arg('shelf_stale_s', SHELF_STALE_S, 'khe stal > nguong -> coi mat ket noi (s)'),
        _arg('camera_topic', '/color/image_raw', 'camera heartbeat de phan biet camera loi voi het pallet'),
        _arg('camera_stale_s', '2.0', 'camera khong co frame qua nguong nay -> Recovery (s)'),
        _arg('pick_sample_wait_s', str(PICK_SAMPLE_WAIT_S),
             'timeout gom mau detect; tu dong dung som khi du mau va pose on dinh (s)'),
        _arg('sample_stable_s', '2.0', 'thoi gian pose pallet phai on dinh lien tuc (s)'),
        _arg('no_pallet_retries', '3', 'so lan scan khong thay pallet truoc khi ve HOME'),
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
        # Rack TF
        _arg('with_rack_tf', WITH_RACK_TF, 'chay rack_tf_publisher cung'),
        _arg('rack_graph_yaml', RACK_GRAPH_YAML,
             'file graph de neo ke (khong hard-link; ke chi phat TF khi co anchor)'),
        _arg('rack_publish_once', RACK_PUBLISH_ONCE,
             'true: kiem tra + phat 1 lan roi thoat (ke fix)'),
        _arg('rack_check_interval_s', RACK_CHECK_INTERVAL_S,
             'chu ki quan sat anchor (s), chi ap khi rack_publish_once=false'),
        # GUI
        _arg('gui', WITH_GUI, 'chay supervisor_gui (nut START/STOP)'),
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
            'run_mode': LaunchConfiguration('run_mode'),
            'mission_manager_node': LaunchConfiguration('mission_manager_node'),
            'pallet_mission_node': LaunchConfiguration('pallet_mission_node'),
            'shelf_topic': LaunchConfiguration('shelf_topic'),
            'shelf_stale_s': pv('shelf_stale_s', float),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_stale_s': pv('camera_stale_s', float),
            'pick_sample_wait_s': pv('pick_sample_wait_s', float),
            'sample_stable_s': pv('sample_stable_s', float),
            'no_pallet_retries': pv('no_pallet_retries', int),
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
        }],
    )

    rack_tf = Node(
        package='amr_mission_manager',
        executable='rack_tf_publisher',
        name='rack_tf_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_rack_tf')),
        parameters=[{
            'graph_yaml_filename': LaunchConfiguration('rack_graph_yaml'),
            'publish_once': pv('rack_publish_once', bool),
            'check_interval_s': pv('rack_check_interval_s', float),
        }],
    )

    gui = Node(
        package='amr_mission_manager',
        executable='supervisor_gui',
        name='supervisor_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription(args + [supervisor, rack_tf, gui])
