"""Pure OpenCV/numpy test of 10 cm B1 marker pose inversion (no ROS required)."""
import math
import cv2
import numpy as np


def T(R, t):
    a = np.eye(4); a[:3,:3] = R; a[:3,3] = t; return a


def Rz(a):
    c,s = np.cos(a),np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def Ry(a):
    c,s = np.cos(a),np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


# Approximate ROS camera optical frame axes expressed in camera_link.
R_body_optical = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
R_slot_marker = np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
T_map_slot = T(np.eye(3), np.array([1.65, 0.0, 0.0]))
T_slot_marker = T(R_slot_marker, np.array([0.0,0.0,0.122]))
T_map_marker = T_map_slot @ T_slot_marker
T_map_base_actual = T(Rz(math.radians(0)), np.array([0.0,0.0,0.0]))
T_base_cam = T(Ry(math.radians(13.2)) @ R_body_optical, np.array([0.0275,0.0197,0.708]))
T_camera_marker = np.linalg.inv(T_map_base_actual @ T_base_cam) @ T_map_marker
rvec,_ = cv2.Rodrigues(T_camera_marker[:3,:3]); tvec=T_camera_marker[:3,3]
L=.10; h=L/2.0
obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]], dtype=np.float64)
K=np.array([[620.591,0,328.200],[0,626.478,241.504],[0,0,1.0]])
D=np.array([0.210883,-0.37682,0.0008015,-0.0042382,0])
image,_=cv2.projectPoints(obj,rvec,tvec,K,D)
image=image.reshape(-1,2)
result=cv2.solvePnPGeneric(obj,image,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
assert result[0], 'PnP failed'
best=None
for rv,tv in zip(result[1],result[2]):
    R,_=cv2.Rodrigues(rv)
    recovered=T_map_marker @ np.linalg.inv(T_base_cam @ T(R,np.asarray(tv).reshape(3)))
    e=np.linalg.norm(recovered[:3,3]-T_map_base_actual[:3,3]); ang=math.atan2(recovered[1,0],recovered[0,0])
    if best is None or e < best[0]: best=(e,ang)
assert best[0] < 1e-5, best
assert abs(best[1]) < 1e-5
print('GEOMETRY OK: base translation error = %.6f m, yaw = %.6f deg' % (best[0],math.degrees(best[1])))
print('Projected corners:', np.round(image,1).tolist())
