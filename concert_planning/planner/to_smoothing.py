from horizon.problem import Problem
from horizon.solvers import Solver

import casadi as cs
import numpy as np

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
    dt = 0.01
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
    w_prox = 1e2
    pb.createResidual('proximity', w_prox*(q - p_trj))

    # minimize acceleration
    pb.createIntermediateResidual('acc', ddq)

    # stay away from rejected q
    reject_w = pb.createParameter('reject_w', nreject)
    reject_q = pb.createParameter('reject_q', nq*nreject)

    reject_cost = 0
    for i in range(nreject):
        qrej = reject_q[(i*nq):(i*nq + nq)]
        sigma = 0.1
        reject_cost += reject_w[i] * cs.exp(-cs.sumsqr(q - qrej)/(2*sigma))

    pb.createIntermediateCost('reject_cost', reject_cost)

    # create solver
    opts = {
        'ilqr.max_iter': 20,
        'ilqr.codegen_enabled': True,
        'ilqr.codegen_workdir': '/tmp/ilqr_drill_planner',
        }

    solv = Solver.make_solver('ilqr', pb, opts)

    rej_i = 0

    # smoothing loop
    for iter in range(max_iter):
        
        # solve TO
        solv.solve()
        pb.getState().setInitialGuess(solv.x_opt)
        pb.getInput().setInitialGuess(solv.u_opt)

        # save smoothed trj
        trj = solv.x_opt[:nq, :]

        # check validity along trajectory
        ok = True

        reject_weight_value = 1e2

        for i in range(N+1):
            
            model.setJointPosition(np.array(trj[:, i]))
            model.update()
            
            if not validity_checker.checkAll():
                print(f'state at {i} not valid, add penalty')
                reject_w[rej_i:rej_i+1].assign(reject_weight_value)
                reject_q[(rej_i*nq):(rej_i*nq + nq)].assign(trj[:, i])
                rej_i = (rej_i + 1) % nreject
                ok = False
        
        if ok:
            print(f'valid smoothed trj found in {iter} iterations')
            return trj
        
        # pump reject weight
        reject_weight_value *= 2.0
        reject_w.assign(reject_w.getValues()*2.0)
        
    return trj