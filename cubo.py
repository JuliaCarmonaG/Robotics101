import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import mstraj

# ========== Definición del robot FANUC CR-7iA/L utilizando parámetros DH ==========
fanuc = rtb.DHRobot([
    rtb.RevoluteDH(d=330, a=50, alpha=-np.pi/2, qlim=[-2.967, 2.967]),  # Articulación 1
    rtb.RevoluteDH(d=0, a=440, alpha=0, qlim=[-1.745, 2.356]),          # Articulación 2
    rtb.RevoluteDH(d=0, a=35, alpha=-np.pi/2, qlim=[-2.094, 2.094]),    # Articulación 3
    rtb.RevoluteDH(d=420, a=0, alpha=np.pi/2, qlim=[-3.141, 3.141]),    # Articulación 4
    rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2, qlim=[-2.094, 2.094]),     # Articulación 5
    rtb.RevoluteDH(d=80, a=0, alpha=0, qlim=[-6.283, 6.283])            # Articulación 6
], name="Fanuc CR-7iA/L")

# ========== Configuración inicial del robot ==========
fanuc.qz = [0, -np.pi/2, 0, 0, 0, 0]  # Postura inicial con el segundo eje en -90°
fanuc.tool = SE3.OA([0, 1, 0], [0, 0, 1])  # Configuración del efector final

# ========== Definición de los puntos de trayectoria ==========
# Se suman 330 mm en Z para alinearlos con el sistema de referencia DH
T_fanuc = np.array([
    [400, 0, 530],  [600, 0, 530],  
    [400, 0, 330],  [600, 0, 330],
    [400, 200, 530], [600, 200, 530],
    [400, 200, 330],  [600, 200, 330]
])

# ========== Generación de la trayectoria ==========
# Se usa 'mstraj' para generar una trayectoria suave entre los puntos definidos
via = np.vstack(T_fanuc)
traj = mstraj(via, qdmax=0.2, dt=0.1, tacc=0.2)
xyz_traj = traj.q  # Extraemos la trayectoria generada

# ========== Resolviendo la Cinemática Inversa ==========
# Se genera la transformación homogénea del efector final para cada punto de la trayectoria
T_tool = [SE3.Trans(p[0], p[1], p[2]) * SE3.OA([0, -1, 0], [1, 0, 0]) for p in xyz_traj]

# Se usa ikine_min en vez de ikine_LM para evitar errores
sol = [fanuc.ikine_min(T, mask=[1, 1, 1, 0, 0, 0]) for T in T_tool]

# Verificamos si las soluciones son válidas
q_sol = np.array([s.q if s.success else fanuc.qz for s in sol])  # Si falla, usa qz

# ========== Simulación ==========
fanuc.plot(q=q_sol, eeframe=True, backend='swift')  # Cambiado a 'swift' para mejor rendimiento