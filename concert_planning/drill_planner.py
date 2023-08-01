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
                      fixed_links=[], 
                      ee_links=['ee_E'])

# hack: fix wheels and base
pln.qmin[6:14] = 0
pln.qmax[6:14] = 0

pln.qmax[:6] = [2, 2, 0, 0, 0, 1.6]
pln.qmin[:6] = -pln.qmax[:6]

# customize environment
pln.vc.planning_scene.addBox(id='wall', size=[0.5, 5, 5], pose=Affine3(pos=[1.5, 0, 0.0]), frame_id='world')
pln.vc.planning_scene.addBox(id='ground', size=[5, 5, 0.1], pose=Affine3(pos=[0.0, 0, -0.80]), frame_id='world')

# start q (usual 6dof pinoblu)
pln.set_start_configuration([0]*14 + [0.0, 0.1, 0.1, -0.1])

# custom goal validity checker
inside_drillata = False

def drillata():

    global inside_drillata

    if inside_drillata:
        return True
    
    print('DRILLATAHHHHHHHHHHHHHHHHH')

    T_pre_drill = pln.nspg.ik.getDesiredPose('ee_E')
    q_pre_drill = pln.model.getJointPosition()

    T_post_drill = T_pre_drill.copy()
    T_post_drill.translation[0] += 0.20
    pln.nspg.ik.setDesiredPose('ee_E', T_post_drill)

    print(T_pre_drill.translation)
    print(T_post_drill.translation)
    
    if not pln.nspg.ik.solve():
        print('ik: drillata fallita')
        pln.nspg.ik.setDesiredPose('ee_E', T_pre_drill)
        pln.model.setJointPosition(q_pre_drill)
        pln.model.update()
        return False

    inside_drillata = True
    q_post_drill_valid = pln.vc.checkAll()
    inside_drillata = False

    if not q_post_drill_valid:
        print('collisioni: drillata fallita')

    pln.nspg.ik.setDesiredPose('ee_E', T_pre_drill)
    pln.model.setJointPosition(q_pre_drill)
    pln.model.update()
    return q_post_drill_valid


# pln.vc.addChecker('drillata', drillata)

# # goal q 
# pln.set_goal_configuration([0]*14 + [2.6, -1.4, 2.1, 1.75, -1.2, -2.0])

# listen to goal from interactive markers (needs marker spawner on namespace and tf_prefix 'planner')
# -> rosrun cartesian_interface marker_spawner _ns:=planner _tf_prefix:=planner
# ctrl+c to stop listening
pln.listen_to_goal()

# pln.vc.removeChecker('drillata')


# # generate goal pose from detected aruco markers
# pln.generate_goal_configuration(
#     {
#         'ee_E': Affine3(pos=[1.13, 0.0, -0.08], rot=[0, 0.7, 0, 0.7])
#     },
#     timeout=10
# )

# plan
plan_ok = False

while not plan_ok:  
    trj, error = pln.plan(timeout=5.0, planner_type='PRMstar', trj_length=10000)
    plan_ok = error == 0


# run twenty times on rviz
pln.play_on_rviz(trj, 5.0)

# send to robot
pln.play_on_robot(trj, 5.0)

rospy.spin()
