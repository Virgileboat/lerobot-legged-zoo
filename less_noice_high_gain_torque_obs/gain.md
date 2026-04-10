# Base actuator gains (no additional scaling).
BASE_KP_HIPZ = 30.0
BASE_KV_HIPZ = 3.0

BASE_KP_HIPX = 40.0
BASE_KV_HIPX = 3.0

BASE_KP_HIP = 60.0
BASE_KV_HIP = 4.0

BASE_KP_KNEE = 60.0
BASE_KV_KNEE = 4.0

BASE_KP_ANKLE = 20.0
BASE_KV_ANKLE = 1.5




for mid in [1,7]:
    robot.set_joint_gains(mid, kp=30, kd=3.0) 

for mid in [5,6,11,12]:
    robot.set_joint_gains(mid, kp=10, kd=0.75) 


robot.set_joint_gains(2, kp=40, kd=3.0) 
robot.set_joint_gains(3, kp=6, kd=4/20) 
robot.set_joint_gains(4, kp=6, kd=4/20) 
robot.set_joint_gains(8, kp=40, kd=3.0) 
robot.set_joint_gains(9, kp=6, kd=4/20)
robot.set_joint_gains(10, kp=6, kd=4/20)
