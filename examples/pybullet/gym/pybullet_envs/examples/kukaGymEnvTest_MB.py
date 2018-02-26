#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

from pybullet_envs.bullet.kukaGymEnv_MB import KukaGymEnv
import time

import numpy as np 

def main():

	environment = KukaGymEnv(renders=True,isDiscrete=False, maxSteps = 10000000)
	
	  
	motorsIds=[]
	#motorsIds.append(environment._p.addUserDebugParameter("posX",0.4,0.75,0.537))
	#motorsIds.append(environment._p.addUserDebugParameter("posY",-.22,.3,0.0))
	#motorsIds.append(environment._p.addUserDebugParameter("posZ",0.1,1,0.2))
	#motorsIds.append(environment._p.addUserDebugParameter("yaw",-3.14,3.14,0))
	#motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))
	
	dv = 0.01 
	motorsIds.append(environment._p.addUserDebugParameter("posX",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("posY",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("posZ",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("yaw",-dv,dv,0))
	motorsIds.append(environment._p.addUserDebugParameter("fingerAngle",0,0.3,.3))
	
	done = False
	i=0
	while (not done):
	    
	  action=[]

	  for motorId in motorsIds:
	    action.append(environment._p.readUserDebugParameter(motorId))
	  
	  # Get item position
	  p = environment._p
	  blockPos,blockOrn=p.getBasePositionAndOrientation(environment.blockUid)

	  # Get gripper position
	  environment._observation = environment._kuka.getObservation()
	  gripperState  = p.getLinkState(environment._kuka.kukaUid,environment._kuka.kukaGripperIndex)
	  gripperPos = gripperState[0]

	  # Get gripper angle
	  currentAngle = environment._kuka.endEffectorAngle

	  # Define target finger angle
	  targetAngle = 1. # Make sure it is not an integer. Eg: 1 -> 1.0
	  targetAngle = blockOrn[-1]-3.14/4# Seems to be a good angle of the lego piece to grab it [completely heuristic].
	  
	  # Define target position
	  targetPosX = blockPos[0]
	  targetPosY = blockPos[1]
	  targetPosZ = blockPos[2]

	  # Trajectory planner (-.-)'
	  n = 750 # Constant for position
	  m = 100 # Constant for finger rotation


	  Dx = targetPosX - gripperPos[0]
	  Dy = targetPosY - gripperPos[1]
	  Dz = targetPosZ - gripperPos[2]
	  Da = targetAngle - currentAngle

	  dx, dy, dz = Dx/n, Dy/n, Dz/n
	  da = Da / m

	  if i % 100 == 0:
	  		print(i)
			print('Da:'+str(da))
			dist = np.sqrt(Dx**2+Dy**2+Dz**2)
			print('Dist:'+str(dist))

	  action = [dx, dy, dz, da, 0.3]

	  # print(blockOrn)
# 
	  # time.sleep(1)
	  i=i+1

	  state, reward, done, info = environment.step2(action)
	  obs = environment.getExtendedObservation()
	  
if __name__=="__main__":
    main()
