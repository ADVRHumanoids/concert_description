from cartesio_planning import planning
from cartesio_planning import validity_check as vc
from cartesio_planning import visual_tools
from cartesio_planning import constraints

from cartesian_interface.pyci_all import *

from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co

from centauro_cartesio import simple_steering

from . import nspg 
from . import manifold 

import numpy as np
import scipy.linalg as la
import scipy.interpolate as interpol
import yaml

import rospy
from std_srvs.srv import Trigger
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
        self.constr = None

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

        # service to play on robot
        self.stop_play_on_rviz = False
        self.srv = rospy.Service(f'/planner/play_on_robot', Trigger, self._stop_play_on_rviz)

    
    def set_constraint(self, constr_config):

        constr_config['joint_limits']['limits'] = dict()
        for i in range(self.model.getJointNum()):
            jname = self.model.getEnabledJointNames()[i]
            constr_config['joint_limits']['limits'][jname] = [float(self.qmin[i]), float(self.qmax[i])]

        cs_str = yaml.dump(constr_config)

        cs_ci = pyci.CartesianInterface.MakeInstance('OpenSot', cs_str, self.model, 0.01)

        self.constr = constraints.CartesianConstraint(cs_ci)

        
    def _stop_play_on_rviz(self, req):
        self.stop_play_on_rviz = True
        return True, 'stopped playing on rviz'


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
        if not self.model.setJointPosition(self.qstart):
            raise RuntimeError('model.setJointPosition failed, invalid q')
        self.model.update()
        self.start_viz.publishMarkers([])
    

    def set_goal_configuration(self, qgoal):
        
        self.qgoal = qgoal

        # note: we project qstart to belong to the manifold passing
        # from qgoal
        self.model.setJointPosition(self.qgoal)
        self.model.update()

        if self.constr is not None:

            self.constr.reset()
            
            self.constr.function(self.qstart)

            try:
                qstart = self.constr.project(self.qstart)
                self.qstart = np.clip(qstart, self.qmin, self.qmax)
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

        # create the goal sampler
        self.nspg = nspg.GoalSampler(self.model, self.dynamic_links, self.static_links, self.vc, self.qmin, self.qmax)

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


        # create the goal sampler
        self.nspg = nspg.GoalSampler(self.model, self.dynamic_links, self.static_links, self.vc, self.qmin, self.qmax)

        # def goal_cb():
        #     self.nspg.rosapi.run()
        #     print(self.model.getJointPosition())
        #     print('goal_found')

        # self.nspg._nspg.setIterationCallback(goal_cb)

        self.model.setJointPosition(self.qstart)
        self.model.update()
        
        while True:
            try:
                self.nspg.rosapi.run()
                success = self.nspg.sample(0.1)
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                # if not found, publish last attempt for debugging purposes
                self.qgoal = self.model.getJointPosition()

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

            if self.constr is None:

                planner = planning.OmplPlanner(
                    self.qmin, self.qmax,
                    yaml.dump(planner_config)
                )

            else:

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
                return self.vc.checkAll()

            planner.setStateValidityPredicate(validity_predicate)

        
        planner = self.planner

        # set start and goal
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
        

    def postprocess_solution(self, trj, T_trj, wheel_names, wheel_radius, velocity_limit):

        model = self.model
        steering = [simple_steering.SimpleSteering(model, w) for w in wheel_names]
        steering_joint_idx = [model.getDofIndex(st.getSteeringJointName()) for st in steering]
        T_trj = 5.0
        dt = T_trj/trj.shape[1]

        # osc
        osc = simple_steering.OmniSteeringController(model, wheel_names, [wheel_radius]*4, dt, 1e9)
        trj_interp = [trj[:, 0]]

        # print
        print(steering_joint_idx)

        for i in range(trj.shape[1] - 1):

            # print(i, flush=True)
            
            qj = trj[:, i]
            vj = (trj[:, i+1] - trj[:, i])/(dt)
            
            model.setJointPosition(qj)
            model.setJointVelocity(vj)
            model.update()

            osc.update(use_base_vel_from_model=True)
            qs = model.getJointPosition()
            
            # trj[:, i+1] = qs

            qprev = trj_interp[-1]
            vs = (qs - qprev)/dt
            n_nodes_to_add = math.ceil(np.max(np.abs(vs) / velocity_limit))

            for k in range(n_nodes_to_add):
                qk = qprev + (k+1)/n_nodes_to_add*(qs - qprev)
                trj_interp.append(qk)

        trj_interp = np.vstack(trj_interp).T
        T_interp = T_trj / trj.shape[1] * trj_interp.shape[1]

        return trj_interp, T_interp


    def play_on_rviz(self, solution, duration):

        # play solution a number of times...
        dt = duration / solution.shape[1]

        self.stop_play_on_rviz = False

        while True:
            for i in range(solution.shape[1]):
                if self.stop_play_on_rviz:
                    return
                q = solution[:, i]
                self.model.setJointPosition(q)
                self.model.update()
                self.plan_viz.publishMarkers([])
                self.vc.checkAll()
                rospy.sleep(dt)
            rospy.sleep(1.0)


    def play_on_robot(self, solution, T):
        if self.robot is None:
            raise Exception('RobotInterface not available')
        self.robot.setControlMode(xbot.ControlMode.Position())
        dt = T / solution.shape[1]
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

        if not self.vc.checkAll():
            print(f'collision detected with link {self.nspg.vc.planning_scene.getCollidingLinks()}')
            check = False

        if self.constr is not None:
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




