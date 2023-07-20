#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import numpy.matlib
# Parameters
sample_time = 1/20              #seconds
cycles = 1
step_interval = 5

points_matrix = np.array([[-1.5,-1.5,1.0],[1.5,-1.5,1.0],[1.5,1.5,1.0],[-1.5,1.5,1.0]])

# Trajectory
duration = cycles*np.size(points_matrix,0)*step_interval

traj = np.zeros((int(duration/sample_time+1),3)) #x y z u v w phi theta thrust phi_cmd theta_cmd

t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

for i in range(1,cycles+1):
    for j in range(1,np.size(points_matrix,0)+1):
        traj_start = (i-1)*np.size(points_matrix,0)*step_interval+(j-1)*step_interval
        traj_end = (i-1)*np.size(points_matrix,0)*step_interval+j*step_interval
        traj[int(traj_start/sample_time):int(traj_end/sample_time),0:3] = np.tile(points_matrix[j-1,:],(int(step_interval/sample_time),1))
traj[-1,0:3] = traj[-2,0:3]

# Write to txt
np.savetxt('points.txt',traj,fmt='%f')