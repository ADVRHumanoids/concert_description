import numpy as np
import yaml 

from . import planner
from cartesio_planning.planning import PositionCartesianSolver
from cartesian_interface.pyci_all import *


class DrillataValidityCheck:

    def __init__(self, pln: planner.Planner, drill_depth, max_torque=None) -> None:
        
        self.model = pln.model
        self._generate_drill_ik()
        self.tau_lim = self.model.getEffortLimits()

        if max_torque is not None:
            self.tau_lim = np.minimum(self.tau_lim, max_torque)

        self.pln = pln

        self.drill_depth = drill_depth
        self.reentrant = False

    
    def is_valid(self) -> bool:

        # protect against recursive call
        if self.reentrant:
            return True
        
        self.reentrant = True
        
        ret = self.can_perform_drill()

        self.reentrant = False

        return ret 
    

    def inside_torque_limits(self) -> bool:
        gq = self.model.computeGravityCompensation()
        if np.any(np.abs(gq[6:]) > self.tau_lim[6:]):
            print('tau lim exceeded')
            return False 
        return True


    def can_perform_drill(self) -> bool:

        # q before drill (generated by goal sampler)
        q_pre_drill = self.pln.model.getJointPosition()

        # reset ik state to pre drill state
        self.ik.reset()

        # set target to post-drill pose
        T_post_drill = self.pln.model.getPose('ee_E')
        T_post_drill.translation[0] += self.drill_depth
        self.ik.setDesiredPose('ee_E', T_post_drill)

        # try to solve ik         
        if not self.ik.solve():
            print('drilling failed due to ik')
            self.pln.model.setJointPosition(q_pre_drill)
            self.pln.model.update()
            return False

        # check solution valid
        q_post_drill_valid = self.pln.vc.checkAll()

        if not q_post_drill_valid:
            print('drill failed due to validity check')
        else:
            print('good post-drill configuration found')

        # restore model state
        self.pln.model.setJointPosition(q_pre_drill)
        self.pln.model.update()
        return q_post_drill_valid
    
    
    def _generate_drill_ik(self):

        ik_cfg = dict()

        ik_cfg['solver_options'] = {'regularization': 1e-2}

        ik_cfg['stack'] = [['ee', 'base'], ['postural']]

        ik_cfg['constraints'] = ['joint_limits']

        ik_cfg['joint_limits'] = {
            'type': 'JointLimits',
        }

        ik_cfg['ee'] = {
            'type': 'Cartesian',
            'distal_link': 'ee_E', 
        }

        ik_cfg['base'] = {
            'type': 'Cartesian',
            'distal_link': 'base_link', 
        }

        ik_cfg['postural'] = {
            'type': 'Postural',
        }

        ik_str = yaml.dump(ik_cfg)

        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot', ik_str, self.model, 1.0)
        self.ik = PositionCartesianSolver(self.ci)