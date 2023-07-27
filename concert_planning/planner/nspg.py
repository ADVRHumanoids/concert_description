from cartesio_planning import NSPG
from cartesio_planning.planning import PositionCartesianSolver

from cartesian_interface.pyci_all import *

import yaml

class GoalSampler:
    def __init__(self, model, dynamic_links, static_links, validity_check):
        ik_str = self._generate_ik_cfg(dynamic_links + static_links)
        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot', ik_str, model, 1.)
        self.rosapi = pyci.RosServerClass(self.ci, 'planner', 'planner', True)
        self.ik = PositionCartesianSolver(self.ci)
        self.model = model
        self.vc = validity_check
        self._nspg = NSPG.GoalSamplerBasic(ik_solver=self.ik, vc_context=self.vc)

    def set_validity_checker(self, vc):
        self.vc = vc
        self._nspg = NSPG.GoalSamplerBasic(ik_solver=self.ik, vc_context=self.vc)

    def set_references(self, links, poses):
        [self.ik.setDesiredPose(link, pose) for link, pose in zip(links, poses)]

    def sample(self, timeout):
        return self._nspg.sample(timeout)

    def _generate_ik_cfg(self, links):
        # write cartesio config
        cfg = dict()

        cfg['solver_options'] = {'regularization': 1e-1}

        cfg['stack'] = [
            list(links), ['postural']
        ]

        cfg['constraints'] = [
            'joint_limits', 
            'velocity_limits'
            ]

        cfg['joint_limits'] = {
            'type': 'JointLimits'
        }

        cfg['velocity_limits'] = {
            'type': 'VelocityLimits',
            'limits': 1.0
        }

        cfg['postural'] = {
            'name': 'postural',
            'type': 'Postural',
            'lambda': 0.01
        }

        for c in links:
            cfg[c] = {
                'type': 'Cartesian',
                'indices': [0, 1, 2, 3, 4, 5],
                'distal_link': c,
            }

        cfg['ee_E']['indices'] = [0, 1, 2, 3, 4, 5]
        # cfg['ee_E']['base_link'] = 'base_link'

        return yaml.dump(cfg)

    