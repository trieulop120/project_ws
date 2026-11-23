"""Pure planar ArUco anchor policy: independent camera observations and bounded AMCL updates."""
import math
from collections import deque
from statistics import median


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def compose_pose(a, b):
    """Compose planar transforms a->b with b->c (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            angle_diff(a[2] + b[2], 0.0))


def inverse_pose(pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (-c * pose[0] - s * pose[1],
            s * pose[0] - c * pose[1], -pose[2])


def map_odom_from_marker(map_base_at_image, odom_base_at_image):
    return compose_pose(map_base_at_image, inverse_pose(odom_base_at_image))


class ObservationWindow:
    """Each sample is one distinct camera timestamp, never a timer heartbeat.

    Store map->odom derived from image-time marker pose and image-time odometry.
    Unlike raw map->base samples, these are comparable while the robot moves.
    """

    def __init__(self, capacity=80):
        self.samples = deque(maxlen=capacity)
        self.last_stamp = None

    def clear(self):
        self.samples.clear()
        # Keep last_stamp: old/duplicate camera timestamps must not become valid
        # merely because the detection window was cleared.

    def add(self, stamp_s, x, y, yaw, rmse_px):
        values = (stamp_s, x, y, yaw, rmse_px)
        if not all(math.isfinite(v) for v in values) or stamp_s <= 0:
            return False
        if self.last_stamp is not None and stamp_s <= self.last_stamp:
            return False
        self.last_stamp = stamp_s
        self.samples.append(values)
        return True

    def evaluate(self, now_s, count, window_s, max_age_s,
                 spread_xy_m, spread_yaw_deg, max_rmse_px, after_stamp=0.0):
        if not self.samples:
            return None, 'no_samples', {}
        newest = self.samples[-1][0]
        age = now_s - newest
        if age < -0.1 or age > max_age_s:
            return None, 'stale_pose', {'sample_age_s': age}
        recent = [s for s in self.samples if s[0] >= newest - window_s
                  and s[0] > after_stamp]
        metrics = {'sample_age_s': age, 'frames': len(recent)}
        if len(recent) < count:
            return None, 'confirming', metrics
        # Validate a recent, consecutive set of distinct detected frames instead
        # of allowing a single older outlier to poison the entire time window.
        # This is particularly important for motion blur while passing a marker.
        recent = recent[-count:]
        metrics['used_frames'] = len(recent)
        xs = [s[1] for s in recent]
        ys = [s[2] for s in recent]
        yaws = [s[3] for s in recent]
        x, y = median(xs), median(ys)
        yaw = math.atan2(sum(math.sin(v) for v in yaws),
                         sum(math.cos(v) for v in yaws))
        spread_xy = max(math.hypot(s[1] - x, s[2] - y) for s in recent)
        spread_yaw = max(abs(math.degrees(angle_diff(s[3], yaw))) for s in recent)
        worst_rmse = max(s[4] for s in recent)
        metrics.update(spread_xy_m=spread_xy, spread_yaw_deg=spread_yaw,
                       max_rmse_px=worst_rmse)
        if spread_xy > spread_xy_m or spread_yaw > spread_yaw_deg:
            return None, 'pose_spread', metrics
        if worst_rmse > max_rmse_px:
            return None, 'reprojection_error', metrics
        return (x, y, yaw, newest), 'confirmed', metrics


def correction_reason(error_xy, error_yaw_deg, age_s, since_last_s,
                      min_xy_m, min_yaw_deg, max_xy_m, max_yaw_deg,
                      max_age_s, min_interval_s, strong_confirmation=False):
    """30 cm / 15 deg requests stronger evidence, not a permanent rejection."""
    values = (error_xy, error_yaw_deg, age_s, since_last_s)
    if not all(math.isfinite(value) for value in values):
        return 'invalid_measurement'
    if age_s < -0.1 or age_s > max_age_s:
        return 'stale_pose'
    if error_xy < min_xy_m and abs(error_yaw_deg) < min_yaw_deg:
        return 'already_aligned'
    if since_last_s < min_interval_s:
        return 'cooldown'
    if (error_xy > max_xy_m or abs(error_yaw_deg) > max_yaw_deg) and not strong_confirmation:
        return 'strong_confirmation_required'
    return 'correct'


def bounded_pose_update(current, target, max_xy_m, max_yaw_deg):
    """Move current map-base pose towards ArUco without exceeding either step cap.

    Returns (requested_pose, fraction, requested_xy_m, requested_yaw_deg).
    This does NOT control robot velocity; it only caps one AMCL /initialpose reset.
    """
    if (max_xy_m <= 0 or max_yaw_deg <= 0 or
            not all(math.isfinite(v) for v in (*current, *target, max_xy_m, max_yaw_deg))):
        raise ValueError('Invalid bounded pose correction')
    dx, dy = target[0] - current[0], target[1] - current[1]
    dxy = math.hypot(dx, dy)
    dyaw = angle_diff(target[2], current[2])
    fraction = min(1.0,
                   max_xy_m / dxy if dxy > 1e-12 else 1.0,
                   math.radians(max_yaw_deg) / abs(dyaw) if abs(dyaw) > 1e-12 else 1.0)
    next_pose = (current[0] + fraction * dx,
                 current[1] + fraction * dy,
                 angle_diff(current[2] + fraction * dyaw, 0.0))
    return (next_pose, fraction, fraction * dxy,
            math.degrees(fraction * abs(dyaw)))


def acknowledgement_matches(observed, before, requested, max_xy_m, max_yaw_deg):
    """Check that AMCL moved *towards* the commanded reset.

    A fixed 12cm/8deg ACK tolerance would incorrectly accept an unchanged AMCL
    pose when a moving correction is limited to only 5cm/2deg.
    """
    if not all(math.isfinite(v) for v in (*observed, *before, *requested)):
        return False
    desired_xy = math.hypot(requested[0] - before[0], requested[1] - before[1])
    desired_yaw_deg = abs(math.degrees(angle_diff(requested[2], before[2])))
    error_xy = math.hypot(observed[0] - requested[0], observed[1] - requested[1])
    error_yaw = abs(math.degrees(angle_diff(observed[2], requested[2])))
    # Use a fraction of the requested step, rather than a global threshold that
    # is larger than the step itself; keep modest margins for AMCL scan noise.
    xy_limit = (min(max_xy_m, max(0.012, 0.45 * desired_xy))
                if desired_xy >= 0.025 else max_xy_m)
    yaw_limit = (min(max_yaw_deg, max(0.6, 0.45 * desired_yaw_deg))
                 if desired_yaw_deg >= 0.7 else max_yaw_deg)
    return error_xy <= xy_limit and error_yaw <= yaw_limit
