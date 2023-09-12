from typing import Any
from horizon.problem import Problem
from horizon.solvers import Solver

import casadi as cs
import numpy as np

import time

class SmootherCallback:
    
    def __init__(self, model, validity_checker, solver) -> None:
        self.model = model
        self.validity_checker = validity_checker
        self.solver = solver 
        self.invalid_q = []
        self.xopt = None 
        self.uopt = None
    
    def __call__(self, fpres) -> Any:

        fpres.print()
        
        if fpres.alpha == 0 or not fpres.accepted:
            return True 
            
                
        # step was accepted by solver, let us check that it
        # leads to a valid state trajectory
        nq = self.model.getJointNum()
        qtrj = fpres.xtrj[:nq, :]
        N = qtrj.shape[1]
        ok = True
        
        for i in range(N):

            qi = qtrj[:, i]
            self.model.setJointPosition(qi)
            self.model.update()
            
            if not self.validity_checker.checkAll():
                print(f'state at k = {i} invalid')
                if ok:
                    self.invalid_q.clear()
                    ok = False
                self.invalid_q.append(qi)

            # time.sleep(0.01)
        
        # step is valid but we previusly rejected a bigger step due to 
        # collisions: we must exit to update the cost function
        if ok and len(self.invalid_q) > 0:
            print(f'alpha = {fpres.alpha} is valid, but a bigger step was prevented by {len(self.invalid_q)} invalid states, will force solver to exit')
            # save valid state trajectory
            self.xopt = fpres.xtrj
            self.uopt = fpres.utrj
            self.solver.force_exit()  
            return True
        
        if not ok and fpres.alpha < 2e-3:
            print(f'step too small, will force solver to exit')
            self.solver.force_exit()  
            return False

        return ok

        

def check_path(qtrj: np.array, model, validity_checker, invalid_q=list()):

    ok = True

    for i in range(qtrj.shape[1]):

        qi = qtrj[:, i]
        model.setJointPosition(qi)
        model.update()
        
        if not validity_checker.checkAll():
            print(f'state at k = {i} invalid')
            if ok:
                invalid_q.clear()
                ok = False
            invalid_q.append(qi)

        time.sleep(0.01)
    
    return ok


def smooth(trj: np.array, model, validity_checker, max_iter=100, nreject=100):

    # build a TO for a second order integrator system
    N = trj.shape[1] - 1
    pb = Problem(N=N)
    nq = model.getJointNum()
    nv = nq
    
    # state model
    q = pb.createStateVariable('q', nq)
    dq = pb.createStateVariable('dq', nq)
    ddq = pb.createInputVariable('ddq', nq)
    xdot = cs.vertcat(
        dq,
        ddq
    )
    dt = 1.0
    pb.setDynamics(xdot)
    pb.setDt(dt)


    vzero = np.zeros((nv, 1))

    # set initial guess from raw trj
    q.setInitialGuess(trj)
    dq.setInitialGuess(np.hstack([np.diff(trj)/dt, vzero]))

    # start and goal from raw trj
    q.setBounds(trj[:, 0], trj[:, 0], nodes=0)
    q.setBounds(trj[:, N], trj[:, N], nodes=N)

    # zero velocity at end points
    dq.setBounds(vzero, vzero, nodes=0)
    dq.setBounds(vzero, vzero, nodes=N)

    # proximal cost
    p_trj = pb.createParameter('trj', nq)
    p_trj.assign(trj)
    w_prox = pb.createParameter('w_prox', 1)
    pb.createResidual('proximity', w_prox*(q - p_trj))

    # minimize acceleration
    pb.createIntermediateResidual('acc', ddq)

    # stay away from rejected q
    reject_w = pb.createParameter('reject_w', nreject)
    reject_q = pb.createParameter('reject_q', nq*nreject)

    reject_cost = 0
    for i in range(nreject):
        qrej = reject_q[(i*nq):(i*nq + nq)]
        sigma = 0.01
        reject_cost += reject_w[i] * cs.exp(-cs.sumsqr(q - qrej)/(2*sigma**2))

    pb.createIntermediateCost('reject_cost', reject_cost)

    # create solver
    opts = {
        'ilqr.max_iter': 100,
        'ilqr.verbose': True,
        'ilqr.codegen_enabled': True,
        'ilqr.codegen_workdir': '/tmp/ilqr_drill_planner',
        }

    solv = Solver.make_solver('ilqr', pb, opts)

    x_opt = pb.getState().getInitialGuess()
    u_opt = pb.getInput().getInitialGuess()

    # check raw trajectory: it must be valid
    if not check_path(x_opt[:nq, :], model, validity_checker):
            raise ValueError('invalid raw trj')
    
    # solve TO increasing the proximal weight until validity is attained
    w_prox.assign(0.1)

    invalid_q = []

    while True:

        print(f'solving TO with w_prox = {w_prox.getValues()[0, 0]}')
        
        if not solv.solve():
            raise RuntimeError('failed to solve TO')
        
        qtrj = solv.x_opt[:nq, :]

        if check_path(qtrj, model, validity_checker, invalid_q):
            print('success!')
            break 
        else:
            w_prox.assign(w_prox.getValues()[0, 0] * 2.0)

    # update initial guess
    x_opt = solv.x_opt
    u_opt = solv.u_opt

    return x_opt[:nq, :]

    # now we try to iteratively decrease the proximal weight in order to improve
    # the acceleration residual

    rej_i = 0
    reject_weight_value = 1e4

    while True:

        reject_q_val = reject_q.getValues()[:, 0]
        reject_w_val = reject_w.getValues()[:, 0]

        print(f'found {len(invalid_q)} invalid states')

        for qc in invalid_q:

            close_gaussian_found = False

            for j in range(nreject):
                qr = reject_q_val[(j*nq):(j*nq + nq)]
                if np.linalg.norm( (qr - qc)/sigma ) < 1:
                    reject_w[j:j+1].assign(reject_w_val[j] * 2.0)
                    close_gaussian_found = True

            if not close_gaussian_found:
                print(f'adding new entry with index {rej_i}')
                reject_w[rej_i:rej_i+1].assign(reject_weight_value)
                reject_q[(rej_i*nq):(rej_i*nq + nq)].assign(qc)
                rej_i = (rej_i + 1) % nreject

        invalid_q.clear()

        print(f'solving TO with w_prox = {w_prox.getValues()[0, 0]}')
        print(f'solving TO with w_reject = {reject_w.getValues()[:, 0]}')

        pb.getState().setInitialGuess(x_opt)
        pb.getInput().setInitialGuess(u_opt)

        if not solv.solve():
            raise RuntimeError('failed to solve TO')
        
        qtrj = solv.x_opt[:nq, :]

        if check_path(qtrj, model, validity_checker, invalid_q):
            print('ok') 
            x_opt = solv.x_opt
            u_opt = solv.u_opt
            w_prox.assign(w_prox.getValues()[0, 0] / 2.0)
        else:
            print('nok') 
    
    exit()
        
    
    # cb = SmootherCallback(model=model, validity_checker=validity_checker, solver=solv)
    # cb.xopt = xtrj_opt
    # cb.uopt = utrj_opt
    # solv.set_iteration_callback(cb)

    rej_i = 0
    reject_weight_value = 1e4

    # smoothing loop
    for iter in range(max_iter):
        
        # solve TO
        cb.invalid_q.clear()

        print(reject_w.getValues()[:, 0].tolist())

        if not solv.solve():
            raise RuntimeError('failed to solve TO')
        
        # check validity
        for i in range(N):

            qi = qtrj[:, i]
            self.model.setJointPosition(qi)
            self.model.update()
            
            if not self.validity_checker.checkAll():
                print(f'state at k = {i} invalid')
                if ok:
                    self.invalid_q.clear()
                    ok = False
                self.invalid_q.append(qi)
        
        # invalid states were hit, add penalization
        print(f'adding penalization term for {len(cb.invalid_q)} states')
        
        reject_q_val = reject_q.getValues()[:, 0]
        reject_w_val = reject_w.getValues()[:, 0]

        for qc in cb.invalid_q:

            close_gaussian_found = False

            for j in range(nreject):
                qr = reject_q_val[(j*nq):(j*nq + nq)]
                if np.linalg.norm( (qr - qc)/sigma ) < 1:
                    reject_w[j:j+1].assign(reject_w_val[j] * 2.0)
                    close_gaussian_found = True

            if not close_gaussian_found:
                print(f'adding new entry with index {rej_i}')
                reject_w[rej_i:rej_i+1].assign(reject_weight_value)
                reject_q[(rej_i*nq):(rej_i*nq + nq)].assign(qc)
                rej_i = (rej_i + 1) % nreject

        pb.getState().setInitialGuess(cb.xopt)
        pb.getInput().setInitialGuess(cb.uopt)


        # pb.getState().setInitialGuess(xtrj_opt)
        # pb.getInput().setInitialGuess(utrj_opt)

        # # save smoothed trj
        # trj = solv.x_opt[:nq, :]

        # # check validity along trajectory
        # ok = True

        

        # for i in range(N+1):
            
        #     model.setJointPosition(np.array(trj[:, i]))
        #     model.update()
            
        #     if not validity_checker.checkAll():
        #         print(f'state at {i} not valid, add penalty')
        #         reject_w[rej_i:rej_i+1].assign(reject_weight_value)
        #         reject_q[(rej_i*nq):(rej_i*nq + nq)].assign(trj[:, i])
        #         rej_i = (rej_i + 1) % nreject
        #         ok = False
        
        # if ok:
        #     print(f'valid smoothed trj found in {iter} iterations')
        #     return trj
        
        # # pump reject weight
        # reject_weight_value *= 2.0
        # reject_w.assign(reject_w.getValues()*2.0)
        
    return trj