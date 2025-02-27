import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialmath.base as smb

np.set_printoptions(suppress=True, precision=4)
    # Crear un robot Puma 560                    
p560=rtb.models.DH.Puma560()
 
p560.plot(p560.qz,block=False)
 
T= sm.SE3.Trans(0.4,0.1,0)

 