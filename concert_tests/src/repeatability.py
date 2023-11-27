from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbi
import numpy as np 
import rospy 
from scipy.io import loadmat, savemat

rospy.init_node('repeatability_test_node')

## create robot ifc
cfg = get_xbot_config(prefix='xbotcore/')
robot = xbi.RobotInterface(cfg)

## TBD go to home
qhome = [.000, 1.069, 0.030, 2.261, 0.009, -0.50]

##
ci = pyci.CartesianInterfaceRos()
ee = ci.getTask('ee_E')
base_link, distal_link = ee.getDistalLink(), ee.getBaseLink()

# poses (big robot)
initial_pose = Affine3(
  pos=[0.6, 0.00, 1.10],
  rot=[0, 0.7, 0, 0.7]
)

trj_dy = 0.4
trj_dz = 0.3

positions = [
    initial_pose.translation + np.array([0, trj_dy, trj_dz]),
    initial_pose.translation + np.array([0, -trj_dy, trj_dz]),
    initial_pose.translation + np.array([0, -trj_dy, -trj_dz]),
    initial_pose.translation + np.array([0, trj_dy, -trj_dz]),
]

trj_segment = 0

pose_ref_list = []
pose_meas_list = []

pose_ref = initial_pose
trj_time = 10.0

while not rospy.is_shutdown():

    try:

        ee.setPoseTarget(pose_ref, trj_time)

        ee.waitReachCompleted(0)

        while True:
            robot.sense()
            qdot = robot.getMotorVelocity()
            # _, s, _ = np.linalg.svd(robot.model().getRelativeJacobian(base_link, distal_link))
            # print(s)
            if np.linalg.norm(qdot, ord=np.inf) < 1e-3:
                print('robot came to a complete stop')
                break
            rospy.sleep(0.1)

        pose_meas = robot.model().getPose(base_link, distal_link)

        pose_ref_list.append(pose_ref.translation)
        pose_meas_list.append(pose_meas.translation)

        trj_segment = (trj_segment + 1) % len(positions)
        pose_ref = Affine3(pos=positions[trj_segment], rot=initial_pose.quaternion)
        trj_time = 2.0

    except rospy.exceptions.ROSInterruptException:

        break


print('saving log')
    
savemat('/tmp/repeatability_test_log.mat', 
        {
            'pose_ref_list': pose_ref_list,
            'pose_meas_list': pose_meas_list,
        })