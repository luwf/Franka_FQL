import numpy as np
import matplotlib.pyplot as plt

jerk_sum_list = []
jerk_mean_list = []
time_list = []
for i in range(0, 26):
    joint0_jerk_data = np.loadtxt('../data/exp2/joint0_jerk_' + str(i+1) + '.txt')
    time_list.append(np.size(joint0_jerk_data) * 0.001)
    jerk_sum_list.append(np.sum(joint0_jerk_data ** 2))
    jerk_mean_list.append(np.sum(joint0_jerk_data**2) / np.size(joint0_jerk_data))

plt.figure(1)
plt.plot(jerk_sum_list)
plt.ylabel('sum_jerk')

plt.figure(2)
plt.plot(jerk_mean_list)
plt.ylabel('mean_jerk')

plt.figure(3)
plt.plot(time_list)
plt.ylabel('time')

plt.show()

# joint0_speed_data = np.loadtxt('../data/exp2/joint0_speed_25.txt')
# joint0_tau_data = np.loadtxt('../data/exp2/joint0_external_tau_25.txt')
# joint0_jerk_data = np.loadtxt('../data/exp2/joint0_jerk_25.txt')
# damping_data = np.loadtxt('../data/exp2/damping_25.txt')

# plt.figure(1)
# plt.plot(joint0_speed_data)
#
# plt.figure(2)
# plt.plot(joint0_tau_data)
#
# plt.figure(3)
# plt.plot(joint0_jerk_data)
#
# plt.figure(4)
# plt.plot(damping_data)
#
# plt.show()