from cartesio_planning import planning
from cartesio_planning import validity_check as vc
from cartesio_planning import visual_tools

from cartesian_interface.pyci_all import *

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co

from . import nspg 
from . import manifold 

import numpy as np
import scipy.linalg as la
import scipy.interpolate as interpol
import yaml
import rospy
from typing import Dict

import math

class Planner:
    
    def __init__(self, name, urdf, srdf, padding=0.0, fixed_links=list(), ee_links=list()):

        # create model interface
        opt = co.ConfigOptions()
        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        opt.set_string_parameter('framework', 'ROS')
        self.model = xbot.ModelInterface(opt)

        # if robot is available, connect to it
        try:
            self.robot = xbot.RobotInterface(opt)
        except:
            self.robot = None
            print('RobotInterface not created')

        # links in contact
        self.static_links = fixed_links

        # moveable end effectors
        self.dynamic_links = ee_links

        # planner manifold
        self.constr = manifold.make_constraint(self.model, self.static_links)

        # define start configuration
        self.qstart = self._generate_start_pose()
        self.model.setJointPosition(self.qstart)
        self.model.update()

        # make validity check (static stability + planning scene collisions)
        self.vc = self._make_vc_context()

        # set padding 
        self.vc.planning_scene.setPadding(padding)

        # marker array visualizers for start pose, goal pose, and plan
        self.start_viz = visual_tools.RobotViz(self.model,
                                               f'/{name}/start',
                                               color=[0, 0, 1, 0.5],
                                               tf_prefix='planner/')
        
        self.start_viz.publishMarkers([])

        self.goal_viz = visual_tools.RobotViz(self.model,
                                              f'/{name}/goal',
                                              color=[0, 1, 0, 0.5],
                                              tf_prefix='planner/')
        
        self.plan_viz = visual_tools.RobotViz(self.model,
                                              f'/{name}/plan',
                                              color=[1, 1, 0.1, 0.5],
                                              tf_prefix='planner/')

        
        # create the goal sampler
        self.nspg = nspg.GoalSampler(self.model, self.dynamic_links, self.static_links, self.vc)

        # joint limits for the planner (clamped between -6 and 6 rad)
        qmin, qmax = self.model.getJointLimits()
        qmin = np.maximum(qmin, -6.0)
        qmax = np.minimum(qmax, 6.0)
        self.qmin = qmin
        self.qmax = qmax

        # start planning scene monitor
        self.vc.planning_scene.startMonitor()
        self.vc.planning_scene.startGetPlanningSceneServer()

        # empty planner
        self.planner = None

        


    def _generate_start_pose(self):

        """
        Start from current robot position. If unavailable, use homing.
        """

        if self.robot is None:
            self.qstart = self.model.getRobotState('home')
        else:
            self.robot.sense()
            qref = self.robot.getPositionReference()
            self.qstart = self.model.mapToEigen(self.robot.eigenToMap(qref))

        return self.qstart
    

    def set_start_configuration(self, qstart):

        self.qstart = qstart
    

    def set_goal_configuration(self, qgoal):
        
        self.qgoal = qgoal

        # note: we project qstart to belong to the manifold passing
        # from qgoal
        self.model.setJointPosition(self.qgoal)
        self.model.update()
        self.constr.reset()
        self.constr.function(self.qstart)

        try:
            self.qstart = self.constr.project(self.qstart)
        except:
            print('failed to project qstart onto goal manifold')

        # publish start markers
        self.model.setJointPosition(self.qstart)
        self.model.update()
        self.start_viz.publishMarkers([])

        # check goal is valid
        valid = True

        if not self.__check_state_valid(self.qgoal):
            print('invalid goal')
            error_color = [178. / 255., 0, 77. / 255., 0.5]
            self.goal_viz.setRGBA(error_color)
            valid = False

        self.goal_viz.publishMarkers([])
        return valid


    def generate_goal_configuration(self, poses: Dict[str, Affine3], timeout=5):

        # set references to goal sampler from detected aruco boxes
        self.nspg.set_references(poses.keys(), 
                                 poses.values())

        # try to obtain a goal pose, timeout 5 secs
        success = self.nspg.sample(timeout)
        qgoal = self.model.getJointPosition()
        qgoal = np.clip(qgoal, self.qmin, self.qmax)
        
        # if valid, set goal
        # if not found, publishes last attempt for debugging purposes
        goal_valid = self.set_goal_configuration(qgoal)

        if not success:
            raise Exception('unable to find a goal configuration!')
        
        if not goal_valid:
            raise Exception('invalid goal configuration was found!')
        
        


    def listen_to_goal(self):

        # self.model.setJointPosition(self.qstart)
        # self.model.update()
        # self.nspg.reset()

        while True:
            try:
                self.nspg.rosapi.run()
                success = self.nspg.sample(0.1)
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                # if not found, publish last attempt for debugging purposes
                self.qgoal = self.model.getJointPosition()

                self.model.update()
                self.constr.reset()

                self.set_goal_configuration(self.qgoal)

                if not self.__check_state_valid(self.qgoal):
                    print('goal not valid ?!?!?!')
                    error_color = [178. / 255., 0, 77. / 255., 0.5]
                    self.goal_viz.setRGBA(error_color)

                self.goal_viz.publishMarkers([])

                if not success:
                    raise Exception('unable to find a goal configuration!')
                
                
                
                self.qgoal = np.clip(self.qgoal, self.qmin, self.qmax)

                return


    def plan(self, planner_type='RRTstar', timeout=1.0, threshold=0.0, trj_length=100):

        if self.planner is None:

            # create planner
            planner_config = {
                'state_space': {
                    'type': 'Atlas', 
                    'rho': 2.0,
                    'epsilon': 0.1,
                    'exploration': 0.8,
                    'alpha': math.pi/16.
                    }
            }

            planner = planning.OmplPlanner(
                self.constr,
                self.qmin, self.qmax,
                yaml.dump(planner_config)
            )

            self.planner = planner

            # define state validator
            def validity_predicate(q):
                self.model.setJointPosition(q)
                self.model.update()
                self.plan_viz.publishMarkers([])
                return self.nspg.vc.checkAll()

            planner.setStateValidityPredicate(validity_predicate)

        
        planner = self.planner

        # set start and goal
        print(f'manifold error: start {self.constr.function(self.qstart)}, goal {self.constr.function(self.qstart)}')
        planner.setStartAndGoalStates(self.qstart, self.qgoal, threshold)

        # solve
        success = planner.solve(timeout, planner_type)

        if success:

            print('attempting to simplify plan..')
            
            if planner.simplifySolutionPath(1.0):
                print('..ok')
            else:
                print('..failed')
            
            planner.interpolateSolutionPath(trj_length)

            solution = np.array(planner.getSolutionPath()).transpose()

            error = la.norm(solution[:, -1] - np.array(self.qgoal), ord=np.inf)
            print(f'Plan error is {error} rad, trajectory length is {solution.shape[1]}')

            return solution, error
        
        else:

            print(f'Plan failed')
            return None, np.inf


    def play_on_rviz(self, solution, ntimes, duration):

        # play solution a number of times...
        dt = duration / solution.shape[1]

        for _ in range(ntimes):
            for i in range(solution.shape[1]):
                q = solution[:, i]
                self.model.setJointPosition(q)
                self.model.update()
                self.plan_viz.publishMarkers([])
                self.vc.checkAll()
                rospy.sleep(dt)


    def play_on_robot(self, solution, T):
        if self.robot is None:
            raise Exception('RobotInterface not available')
        self.robot.setControlMode(xbot.ControlMode.Position())
        cin = input('send solution? (y/n)')
        dt = T / solution.shape[1]
        if cin.lower() == 'y':
            for i in range(solution.shape[1]):
                q = solution[6:, i]
                self.robot.setPositionReference(q)
                self.robot.move()
                rospy.sleep(dt)


    def interpolate(self, solution, dt, max_qdot, max_qddot):

        qsize = solution.shape[0]
        nknots = solution.shape[1]

        seg_durs = []
        seg_vels = []

        safety_factor = 1.2

        for i in range(nknots - 1):
            tk = np.abs(((solution[:, i + 1] - solution[:, i]) / max_qdot)).max() * safety_factor
            tk = max(tk, 0.001)
            seg_vels.append((solution[:, i + 1] - solution[:, i]) / tk)
            seg_durs.append(tk)

        for i in range(len(seg_vels) - 1):
            acc_max = np.abs((seg_vels[i + 1] - seg_vels[i]) / seg_durs[i]).max()
            if acc_max > max_qddot:
                seg_durs[i] *= acc_max / max_qddot * safety_factor

        seg_durs.insert(0, 0)

        times = np.cumsum(seg_durs)


        interpolators = []
        for i in range(qsize):
            inter = interpol.UnivariateSpline(times, solution[i, :], s=0)
            interpolators.append(inter)

        nsamples = int(np.sum(seg_durs) / dt)
        solution_interp = np.zeros((qsize, nsamples))

        for i in range(len(interpolators)):
            for j in range(nsamples):
                solution_interp[i, j] = interpolators[i](dt * j)

        return solution_interp, [dt * j for j in range(nsamples)]
    

    def  __check_state_valid(self, q):
        check = True

        self.model.setJointPosition(q)
        self.model.update()

        if not self.nspg.vc.checkAll():
            print(f'collision detected with link {self.nspg.vc.planning_scene.getCollidingLinks()}')
            check = False

        error = self.constr.function(q)
        if np.linalg.norm(error) > 0.01:
            print('configuration not on manifold')
            check = False

        return check


    def _make_vc_context(self):
        _planner_config = dict()
        _planner_config['state_validity_check'] = ['collisions']
        _planner_config['collisions'] = {'type': 'CollisionCheck', 'include_environment': 'true'}
        vc_context = vc.ValidityCheckContext(yaml.safe_dump(_planner_config), self.model)
        return vc_context




