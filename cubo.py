import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

# ========== FANUC CR-7iA/L ==========
fanuc = rtb.DHRobot([
    rtb.RevoluteDH(d=330, a=50, alpha=-np.pi/2, qlim=[-2.967, 2.967]),
    rtb.RevoluteDH(d=0, a=440, alpha=0, qlim=[-1.745, 2.356]),
    rtb.RevoluteDH(d=0, a=35, alpha=-np.pi/2, qlim=[-2.094, 2.094]),
    rtb.RevoluteDH(d=420, a=0, alpha=np.pi/2, qlim=[-3.141, 3.141]),
    rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2, qlim=[-2.094, 2.094]),
    rtb.RevoluteDH(d=80, a=0, alpha=0, qlim=[-6.283, 6.283])
], name="Fanuc CR-7iA/L")

# Configuración del efector final
fanuc.tool = SE3.OA([0, 1, 0], [0, 0, 1])
fanuc.qz = [0, 0, 0, 0, 0, 0]

# Puntos de trayectoria
T_fanuc = np.array([
    [400, 0, 200],  [600, 0, 200],
    [400, 0, 0],    [600, 0, 0],
    [400, 200, 200], [600, 200, 200],
    [400, 200, 0],  [600, 200, 0]
])

# Generación de la trayectoria
via = np.vstack(T_fanuc)
xyz_traj = rtb.mstraj(via, qdmax=[0.5, 0.5, 0.5], dt=0.02, tacc=0.2).q

# Cinemática inversa
T_tool = SE3.Trans(xyz_traj) * SE3.OA([0, -1, 0], [1, 0, 0])
sol = fanuc.ikine_LM(T_tool, mask=[1, 1, 1, 0, 0, 0])

# Simulación
fanuc.plot(q=sol.q, eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)
