from matplotlib import pyplot as plt
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from codigoVerTrayect import plot_robot_trajectory

# Conversión de grados a radianes
deg_to_rad = np.pi / 180

# Definir el modelo DH del ABB IRB 580/16
abb_irb580 = rtb.DHRobot([
    rtb.RevoluteDH(d=0.630, a=0, alpha=-90 * deg_to_rad),   # J1
    rtb.RevoluteDH(d=0, a=1.000, alpha=0),                 # J2
    rtb.RevoluteDH(d=0, a=0, alpha=-90 * deg_to_rad),      # J3
    rtb.RevoluteDH(d=1.620, a=0, alpha=-30 * deg_to_rad),  # J4
    rtb.RevoluteDH(d=0, a=0, alpha=60 * deg_to_rad),       # J5
    rtb.RevoluteDH(d=0, a=0, alpha=-30 * deg_to_rad)       # J6
], name="ABB IRB 580/16")

# Ajustar la base en el suelo
abb_irb580.base = SE3.Trans(0, 0, 0)  

# Ajustar la herramienta (TCP) para que apunte correctamente
abb_irb580.tool = SE3.Rx(np.pi) * SE3.Rz(np.pi/2)

# Configuración inicial
abb_irb580.qz = [0, -90 * deg_to_rad, 0, 0, 0, 0]  

# Imprimir el modelo del robot
print(abb_irb580)

# Graficar con límites adecuados
abb_irb580.plot(q=abb_irb580.qz, limits=[-2, 2, -2, 2, -0.5, 2], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)
T=np.array([[1, 0.5, 1.8005],[1.5, 0.5, 1.8005],[1.5, 0.5, 1.3005], [1, 0.5, 1.3005],
   [1, 0.5, 1.8005], [1, 0, 1.8005],[1, 0, 1.3005],[1, 0.5, 1.3005],[1, 0, 1.3005],[1.5, 0, 1.3005],
    [1.5, 0, 1.8005],[1, 0, 1.8005],
    [1.5, 0, 1.8005],[1.5, 0.5, 1.8005],[1.5, 0.5, 1.3005],[1.5, 0, 1.3005]
])

#metodo 1
sol=abb_irb580.ikine_LM(SE3(T), q0=abb_irb580.qz)
#fanuc.plot(sol.q, limits=[-1, 1, -1, 1, -0.1, 1], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True, dt=0.5)
p_lim=[-1, 1, -1, 1, -0.15, 1.5]
plot_robot_trajectory(
    robot=abb_irb580,
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
sol=abb_irb580.ikine_LM(T_tool,'lu')
print(sol.success)

#fanuc.plot(q=sol.q, limits=[-1, 1, -1, 1, -0.1, 1], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)
plot_robot_trajectory(
    robot=abb_irb580,
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

Joints = [[90, 0, 0, 90, 0, 0], [0, 90, 0, 0, 0, 90], [90, 0, 0, 90, 90, 90], [90, 0, 0, 90, 90, 0], [0, 0, 0, 90, 0, 0]]
Joints = np.deg2rad(Joints)
tray_final = []

for i in range(len(Joints) - 1):
    tray_parcial = rtb.jtraj(Joints[i], Joints[i + 1], 10)
    tray_final.extend(tray_parcial.q)
tray_final=np.array(tray_final)
#fanuc.plot(q=tray_final, limits=[-1, 1, -1, 1, -0.1, 1], eeframe=True, backend='pyplot', shadow=True, jointaxes=True, block=True)
plot_robot_trajectory(
    robot=abb_irb580,
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
