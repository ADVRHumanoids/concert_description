#!/usr/bin/env python3

import rospy
import numpy as np
import scipy.io
import math
import yaml

from cartesian_interface.pyci_all import *
import cartesian_interface.roscpp_utils as roscpp
from moveit_compute_default_collisions import pymcdc

from planner import planner
from planner.drill_validity_check import DrillataValidityCheck

from centauro_cartesio import simple_steering

np.set_printoptions(precision=2, suppress=True)

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
                      fixed_links=[], 
                      ee_links=['ee_E'])

# hack: fix wheels and base
pln.qmin[6:14] = 0
pln.qmax[6:14] = 0

pln.qmax[:6] = [5, 5, 0, 0, 0, 1.6]
pln.qmin[:6] = -pln.qmax[:6]

# customize environment
pln.vc.planning_scene.addBox(id='wall', size=[0.5, 5, 5], pose=Affine3(pos=[1.5, 0, 0.0]), frame_id='world')
pln.vc.planning_scene.addBox(id='ground', size=[5, 5, 0.1], pose=Affine3(pos=[0.0, 0, -0.80]), frame_id='world')
pln.vc.planning_scene.addCylinder('cestino', 0.3, 1.0, Affine3(pos=[0, 0, -0.3]), 'world')

# start q (usual 6dof pinoblu)
pln.set_start_configuration([0, 1.0, 0, 0, 0, 0.0] + [0]*8 + [0.0, 1.4, 0, 2.8, 0, 1.4])

# custom goal validity checker
drill_checker = DrillataValidityCheck(pln=pln, drill_depth=0.20, max_torque=150.0)

pln.vc.addChecker('drillata', drill_checker.is_valid)

# # goal q 
# pln.set_goal_configuration([0]*14 + [2.6, -1.4, 2.1, 1.75, -1.2, -2.0])

# # listen to goal from interactive markers (needs marker spawner on namespace and tf_prefix 'planner')
# # -> rosrun cartesian_interface marker_spawner _ns:=planner _tf_prefix:=planner
# # ctrl+c to stop listening
# pln.listen_to_goal()

# generate goal pose from detected aruco markers
pln.generate_goal_configuration(
    {
        'ee_E': Affine3(pos=[1.0, -0.70, -0.50], rot=[0, 0.7, 0, 0.7])
    },
    timeout=10
)

# remove this checker for planning
pln.vc.removeChecker('drillata')

# plan
plan_ok = False

while not plan_ok:  
    trj, error = pln.plan(timeout=5.0, planner_type='RRTConnect', trj_length=500)
    plan_ok = error == 0

# post process trj

wheels = [f'wheel_{l}' for l in ['A', 'B', 'C', 'D']]
T_trj = 5.0
trj_interp, T_interp = pln.postprocess_solution(trj, T_trj, wheels, wheel_radius=0.16, velocity_limit=2.0)

scipy.io.savemat('/tmp/drill_planner.mat', 
                 {
                     'trj': trj,
                     'trj_interp': trj_interp,
                     'T_trj': T_trj,
                     'T_trj_interp': T_interp
                 })


# run on rviz
pln.play_on_rviz(trj_interp, T_interp)

# send to robot
pln.play_on_robot(trj, 5.0)

rospy.spin()
