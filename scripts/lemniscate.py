#!/usr/bin/env python3

import numpy as np
import rospy,rospkg

rospy.init_node('generate_traj_node',anonymous=True)
frequency = rospy.get_param('/airo_control_node/fsm/fsm_frequency')

rospack = rospkg.RosPack()
package_path = rospack.get_path('airo_trajectory')
output_path = package_path + '/scripts/lemniscate.txt'

# Parameters
sample_time = 1/frequency      # seconds
duration = 30;                      #seconds

amp = 5
frq = 1

x0 = 0
y0 = 0
z0 = 1

# Trajectory
traj = np.zeros((int(duration/sample_time+1),6)) #x y z u v w
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = amp*np.cos(t*frq)+x0
traj[:,1] = amp*np.sin(t*frq)*np.cos(t*frq)+y0
traj[:,2] = z0
traj[:,3] = -amp*frq*np.sin(t*frq)
traj[:,4] = amp*frq*np.cos(t*2*frq)
traj[:,5] = 0

# Write to txt
np.savetxt('lemniscate.txt',traj,fmt='%f')