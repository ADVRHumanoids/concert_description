#!/usr/bin/env python3

import rospy
import numpy as np
import scipy.io

from cartesian_interface.pyci_all import *
import cartesian_interface.roscpp_utils as roscpp
from moveit_compute_default_collisions import pymcdc

from planner import planner


# initialize rospy and roscpp
rospy.init_node('planner_node', disable_signals=True)
roscpp.init('planner_node', [])

# get config object (contains urdf, srdf)
model_cfg = get_xbot_config(prefix='xbotcore/')

# generate acm and write it into srdf
acm = pymcdc.MoveitComputeDefaultCollisions()
acm.initFromString(model_cfg.get_urdf(), model_cfg.get_srdf(), False)
acm.computeDefaultCollisions(int(1e5))
# acm.printDisabledCollisions()
srdf_with_acm = acm.getXmlString()

# construct planner class
pln = planner.Planner(name='concert', 
                      urdf=model_cfg.get_urdf(), 
                      srdf=srdf_with_acm, 
                      fixed_links=['base_link'], 
                      ee_links=['ee_E'])

# hack: fix wheels
pln.qmin[6:14] = 0
pln.qmax[6:14] = 0

# customize environment
pln.vc.planning_scene.addBox('lasbarra', [0.1, 2.1, 0.1], Affine3(pos=[0.5, 0, 0.7]), 'base_link')
pln.vc.planning_scene.addBox('cilindrozzo', [0.15, 0.15, 0.2], Affine3(pos=[0, 0, 0.15]), 'ee_E', 'ee_E')
pln.vc.planning_scene.addCylinder('cestino', 0.3, 1.0, Affine3(pos=[0, 0.8, -0.3]), 'base_link')

# start q (usual 6dof pinoblu)
pln.set_start_configuration([0]*14 + [0, -1.4, 0, 1.0, 0, -0.6])

# goal q 
pln.set_goal_configuration([0]*14 + [2.6, -1.4, 2.1, 1.75, -1.2, -2.0])

# # listen to goal from interactive markers (needs marker spawner on namespace and tf_prefix 'planner')
# # -> rosrun cartesian_interface marker_spawner _ns:=planner _tf_prefix:=planner
# # ctrl+c to stop listening
# pln.listen_to_goal()

# # generate goal pose from detected aruco markers
# pln.generate_goal_configuration(
#     {
#         'ee_E': Affine3(pos=[-0.2, -0.1, 0.2], rot=[1, 0, 0, 0])
#     },
#     timeout=1
# )

# plan
plan_ok = False 

while not plan_ok:  
    trj, error = pln.plan(timeout=1.0, planner_type='RRTConnect', trj_length=1000)
    plan_ok = error == 0


# run twenty times on rviz
pln.play_on_rviz(trj, 20, 5.0)

# send to robot
pln.play_on_robot(trj, 5)

rospy.spin()
