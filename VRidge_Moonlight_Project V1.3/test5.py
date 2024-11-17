# import numpy as npo
# alphaSOld = np.nan
# r = 0

# while doMeasure:
#     alphaS = getAngle() # returns angle in range [-pi, pi)

#     if not np.isnan(alphaSOld):
#         if alphaS - alphaSOld < -math.pi:
#             r += 1
#         elif alphaS - alphaSOld > math.pi:
#             r -= 1

#     alphaM = alphaS + r * 2 * math.pi
#     kf.update(alphaM)

#     alphaSOld = alphaS

import math
pi = 3.141592653589793
print(math.pi)

print(math.radians(178))

# // reduce the angle to [-359; +359], +-360 corresponds to 0
angle = pi
angle =  angle % 2*pi; 

# // normalize from [-359; +359] to [-179; +180]
if (angle <= -pi):angle += 2*pi
if (angle > pi):angle -= 2*pi

print(math.degrees(angle))

# for i in range(180):
#     yvalues = i * math.pi/180, i
#     print(yvalues)


# import numpy as np
# from numpy import random
# from numpy.random import randn
# import matplotlib.pyplot as plt
# from filterpy.kalman import KalmanFilter
# import kf_book.book_plots as bp


# def plot_rts(noise, Q=0.001, show_velocity=False):

#     random.seed(123)
#     fk = KalmanFilter(dim_x=2, dim_z=1)
#     fk.x = np.array([0., 1.])  # state (x and dx)
#     fk.F = np.array([[1., 1.], [0., 1.]])  # state transition matrix
#     fk.H = np.array([[1., 0.]])  # Measurement function
#     fk.P = 10.  # covariance matrix
#     fk.R = noise  # state uncertainty
#     fk.Q = Q  # process uncertainty
#     # create noisy data
#     # zs = np.asarray([t + randn()*noise for t in range(40)])
#     zs = [-2.8312382712715762, -2.8265694995060535, -2.8258980068414297, -2.8458181648550425, -2.7912555389693656, -2.857924377446126, -2.842849298047664, -2.807452713846611, -2.852888428207214, -2.811522953526229, -2.7712594242760957, -2.765968303470298, -2.842433677769943, -2.820533727210549, -2.807062359565653, -2.8010485245341252, -2.803813714052873, -2.804700950330332, -2.811604626552085, -2.8077284851839606, -2.8314253286718154, -2.8622323048511347, -2.8196201259519587, -2.8443564555637852, -2.8159642528084476, -2.827326148522156, -2.831833968938098, -2.850436005032774, -2.8527956646249177, -2.876772938853035]
#     print(zs)
#     # filter data with Kalman filter, than run smoother on it

#     mu, cov, _, _ = fk.batch_filter(zs)
#     M, P, C, _ = fk.rts_smoother(mu, cov)
#     print(M)
#     # plot data
#     if show_velocity:
#         index = 1
#         print('gu')
#     else:
#         index = 0
#     if not show_velocity:
#         bp.plot_measurements(zs, lw=1)
#         plt.plot(M[:, index], c='b', label='RTS')
#         plt.plot(mu[:, index], c='g', ls='--', label='KF output')
#     if not show_velocity:
#         N = len(zs)
#         plt.plot([0, N], [0, N], 'k', lw=2, label='track')
#         plt.legend(loc=4)
#         plt.show()


# plot_rts(noise=7,Q=0.1)
