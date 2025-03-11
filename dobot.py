from matplotlib import pyplot as plt
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from codigoVerTrayect import plot_robot_trajectory

# Conversión de grados a radianes
deg_to_rad = np.pi / 180

# Definir el modelo DH del segundo robot basado en la tabla DH
robot_alt = rtb.DHRobot([
    rtb.RevoluteDH(d=0.0674, a=0, alpha=90 * deg_to_rad),   # J1
    rtb.RevoluteDH(d=0.0644, a=0.274, alpha=180 * deg_to_rad), # J2
    rtb.RevoluteDH(d=0, a=0.230, alpha=180 * deg_to_rad),   # J3
    rtb.RevoluteDH(d=0.058, a=0, alpha=-90 * deg_to_rad),   # J4
    rtb.RevoluteDH(d=0.116, a=0, alpha=90 * deg_to_rad),    # J5
    rtb.RevoluteDH(d=0.0565, a=0, alpha=0)                 # J6
], name="Alternative Robot")

# Ajustar la base en el suelo
robot_alt.base = SE3.Trans(0, 0, 0)  

# Ajustar la herramienta (TCP) para que apunte correctamente
robot_alt.tool = SE3.Rx(np.pi) * SE3.Rz(np.pi/2)

# Configuración inicial
robot_alt.qz = [0, 90 * deg_to_rad, 0, -90 * deg_to_rad, 0, 0]  
robot_q0=[np.deg2rad(-10.2), np.deg2rad(97.9), np.deg2rad(80.8), np.deg2rad(-30.2), np.deg2rad(90), np.deg2rad(91)]
# Imprimir el modelo del robot
print(robot_alt)
#modo teach
robot_alt.teach(robot_alt.qz,block=True)
# Graficar con límites adecuados
robot_alt.plot(q=robot_alt.qz, limits=[-1, 1, -1, 1, -0.5, 1.5], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)

T=np.array([[.346, -.1283, .343],  [.346, -.1283, .243],[.346, -.0283, .243],  [.346, -.0283, .343],[.346, -.1283, .343],
    [.246, -.1283, .343],  [.246, -.1283, .243], [.346, -.1283, .243], [.246, -.1283, .243],  [.246, -.0283, .243],
     [.246, -.0283, .343],  [.346, -.0283, .343], [.246, -.0283, .343],
    [.346, -.0283, .343],[.346, -.0283, .243],[.246, -.0283, .243],[.246, -.0283, .343], [.246, -.1283, .343]
])

#metodo 1
sol=robot_alt.ikine_LM(SE3(T), q0=robot_alt.qz)
#fanuc.plot(sol.q, limits=[-1, 1, -1, 1, -0.1, 1], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True, dt=0.5)
p_lim=[-.2, .2, -.2, .2, -0.5, .2]
plot_robot_trajectory(
    robot=robot_alt,
    q_trajectory=sol.q,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.2,
    block=True
)
#metodo2
via=np.empty((0,3))
for punto in T:
    xyz=np.array(punto)
    via=np.vstack((via, xyz))
xyz_traj=rtb.mstraj(via, qdmax=[0.2,0.2,0.2], dt=0.05, tacc=0.2).q
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot(xyz_traj[:,0], xyz_traj[:,1], xyz_traj[:,2])
ax.scatter(xyz_traj[0,0], xyz_traj[0,1], xyz_traj[0,2], color='r', marker='*')
ax.scatter(xyz_traj[-1,0], xyz_traj[-1,1], xyz_traj[-1,2], color='g', marker='o')
plt.show()
T_tool= SE3.Trans(xyz_traj)
sol=robot_alt.ikine_LM(T_tool,q0=robot_q0,ilimit=1000, slimit=20)
print(sol.success)

#fanuc.plot(q=sol.q, limits=[-1, 1, -1, 1, -0.1, 1], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)
plot_robot_trajectory(
    robot=robot_alt,
    q_trajectory=sol.q,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.5,
    block=True
)

Joints = [[90, 0, 0, 90, 0, 0], [0, 90, 0, 0, 0, 90], [90, 0, 0, 90, 90, 90], [90, 0, 0, 90, 90, 0], [0, 0, 0, 90, 0, 0]]
Joints = np.deg2rad(Joints)
tray_final = []

for i in range(len(Joints) - 1):
    tray_parcial = rtb.jtraj(Joints[i], Joints[i + 1], 10)
    tray_final.extend(tray_parcial.q)
tray_final=np.array(tray_final)
#fanuc.plot(q=tray_final, limits=[-1, 1, -1, 1, -0.1, 1], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)
plot_robot_trajectory(
    robot=robot_alt,
    q_trajectory=tray_final,
    limits=p_lim,
    eeframe=True,
    jointaxes=False,
    shadow=True,
    drawing_mode='continuous',  # o 'segments' si prefieres
    traj_color='r',             # Color de la trayectoria completa
    drawing_color='b',          # Color del trazo principal
    dt=0.2,
    block=True
)
