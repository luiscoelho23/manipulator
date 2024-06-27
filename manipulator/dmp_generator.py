#!/usr/bin/python3

# import math
import sys
import numpy as np
import os
import matplotlib.pyplot as plt

# assert(len(sys.argv) > 1)
# sys.path.append(sys.argv[1])
sys.path.append('/home/luisc/workspaces/ws_manipulator/build/mplibrary')
import pymplibrary as motion
sys.path.append('/home/luisc/workspaces/ws_manipulator/build/mplearn')
import pymplearn as learn

# load DMP learning configuration
try:
    # config = learn.regression.Config.load('discrete_dmp_regression.yaml')
    config = learn.regression.Config.load("/home/luisc/workspaces/ws_manipulator/src/manipulator/resource/dmp/discrete.yaml")

except Exception as err:
    print('Usage: python ' + os.path.basename(__file__) + ' <config>')
    exit(1)
    # config = learn.regression.Config()  # default parameters
    pass

# config.plot = True
# print(config.n_basis)

# load reference data to learn
reference = np.loadtxt('/home/luisc/workspaces/ws_manipulator/src/manipulator/resource/dmp/6dmp.csv', delimiter=',')
# reference = reference[:, 1:]  # discard first column (sample index)
n_samples, n_dims = reference.shape

# assign phase values
# configured timestep used to estimate time stamps
# @note alternatively, reference file can be used if time stamps are available
pvals = [0.0] * n_samples
tspan = n_samples * config.rollout.integration_timestep
phase = motion.LinearPacer(1.0)
phase.limits.lower = config.phase_bounds[0]
phase.limits.upper = config.phase_bounds[1]
phase.pace = (phase.limits.upper - phase.limits.lower) / tspan
phase.value = phase.limits.lower
for idx in range(n_samples):
    pvals[idx] = phase.value
    phase.update(config.rollout.integration_timestep)

print('Phase values: [' + str(pvals[0]) + ' ... ' + str(pvals[-1]) + ']')

# initialize & strucute DMP system
# create multiple primitives in bulk
# @note alternatively, primtives could be created & added to library one by one
library = motion.Library()
library.primitives = motion.Primitive.GaussianPrimitiveLibrary(config.n_primitives, config.n_basis, phase.limits.lower, phase.limits.upper, 0.15 * motion.kernel.Gaussian.scaleWidth(config.n_basis))

# plot basis (for debug)
#fig, ax = plt.subplots(constrained_layout=True)
#ax.set_xlim(config.phase_bounds)
#for b in library.primitives[0]:
#    bs = []
#    for p in pvals:
#        bs.append(b(p))
#    ax.plot(pvals, bs)
#plt.show()

# initialize & parametrize skill / primitive combination
skill = motion.Skill(library.primitives, n_dims)
skill.setWeights(motion.mat(np.identity(n_dims)))  # identity matrix as primitive mixing (single primitive per dimension)
# initialize and parametrize DMP policy
policy = motion.DMP(n_dims, skill)
# parametrize DMP

for idx, dim in enumerate(policy): 
    dim.goal = learn.regression.getDMPBaseline(reference[:, idx], config.baseline_method)
    dim.parameters = config.damping
    dim.value = dim.goal   # reset state to baseline

# run optimization
# @note multifit() runs DMP regression for each *row* in reference data, need to transpose reference data
# @note motion.mat() to wrap numpy arrays when passing to mplearn/mplibrary types
sols = learn.regression.fitPolicy(motion.mat(reference.transpose()), pvals, policy, "", config)

# normalize DMP temporal scale
# @note optional, otherwise phase slope would need to be stored in target file
policy.tscale(1.0 / phase.pace)

# export to file
library.skills.add(skill)
library.policies.add(policy)
motion.mpx.save(library, '/home/luisc/workspaces/ws_manipulator/src/manipulator/resource/dmp/dmp_output/dmp.mpx')