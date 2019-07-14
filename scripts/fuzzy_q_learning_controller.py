#!/usr/bin/env python
import rospy
from franka_ros.msg import franka_state
from franka_ros.msg import control_cmd

import FuzzySet
import StateVariable
import FQL
import FIS
import time
from Admittance_control import Admittance_control

pub = rospy.Publisher('control_node', control_cmd, queue_size=10)
franka_cmd = control_cmd()
count = 0
reward_list = []
reward_sum = 0
iter_num = 0
action = 0
time_count = 0
start_time = 0
end_time = 0
episode_time = 20
episode_count = 1

f1 = open('src/franka_ros/data/joint0_speed_' + str(episode_count) + '.txt','w')
f2 = open('src/franka_ros/data/joint0_jerk_' + str(episode_count) + '.txt','w')
f3 = open('src/franka_ros/data/joint0_external_tau_' + str(episode_count) + '.txt','w')
f4 = open('src/franka_ros/data/damping_' + str(episode_count) + '.txt','w')

# Create FIS
x1 = StateVariable.InputStateVariable(FuzzySet.Trapeziums(-100, -0.525, -0.475, -0.275),
                                      FuzzySet.Triangles(-0.5, -0.25, 0),
                                      FuzzySet.Triangles(-0.25, 0, 0.25),
                                      FuzzySet.Triangles(0, 0.25, 0.5),
                                      FuzzySet.Trapeziums(0.275, 0.475, 0.525, 100))
x2 = StateVariable.InputStateVariable(FuzzySet.Trapeziums(-100, -5.25, -4.75, -2.75),
                                      FuzzySet.Triangles(-5, -2.5, 0),
                                      FuzzySet.Triangles(-2.5, 0, 2.5),
                                      FuzzySet.Triangles(0, 2.5, 5),
                                      FuzzySet.Trapeziums(2.75, 4.75, 5.25, 100))
fis = FIS.Build(x1,x2)

# Create Model
model = FQL.Model(gamma = 0.95, alpha = 0.05 , ee_rate = 0.1, past_weight = 0.9, q_initial_value = 'zero',
                  action_set_length = 3, fis = fis)
controller = Admittance_control()
controller.__int__()

def callback(franka_msg):
    global count
    global reward_list
    global reward_sum
    global iter_num
    global action
    global time_count
    global start_time
    global end_time
    global episode_count
    global f1, f2, f3, f4

    if time_count == 0:
        start_time = time.time()
    end_time = time.time()

    joint0_speed = franka_msg.joint_speed[0]
    joint0_external_tau = franka_msg.joint_tau[0] - franka_msg.joint_initial_tau[0] \
                              - franka_msg.joint_coriolis[0] - franka_msg.joint_gravity[0]
    joint0_jerk = franka_msg.joint_jerk[0]

def callback(franka_msg):
    global count
    global reward_list
    global reward_sum
    global iter_num
    global action
    global time_count
    global start_time
    global end_time
    global episode_count
    global f1, f2, f3, f4

    if time_count == 0:
        start_time = time.time()
    end_time = time.time()

    if (end_time - start_time < episode_time):
        time_count += 1

        joint0_speed = franka_msg.joint_speed[0]
        joint0_external_tau = franka_msg.joint_tau[0] - franka_msg.joint_initial_tau[0] \
                              - franka_msg.joint_coriolis[0] - franka_msg.joint_gravity[0]
        joint0_jerk = franka_msg.joint_jerk[0]

        # back to initial state
        if(abs(joint0_external_tau) < 1):
            franka_cmd.control_flag = 0
            controller.__int__()
            count = 0
            reward_list = []
            reward_sum = 0
            iter_num = 0
            action = 0

        # FQL
        else:
            franka_state = [joint0_speed, joint0_external_tau]

            if count < 10:
                reward = -(joint0_jerk * joint0_jerk)
                reward_list.append(reward)
                count += 1
            else:
                reward_sum = sum(reward_list)
                count = 0
                reward_list = []

            if iter_num == 0:
                action = model.get_initial_action(franka_state)
            else:
                action = model.run(franka_state, reward_sum)
            iter_num += 1

            joint0_speed_cmd = controller.admittance_controller(-joint0_external_tau, action)
            if joint0_speed_cmd > 2.8973:
                joint0_speed_cmd = 2.8973
            elif joint0_speed_cmd < -2.8973:
                joint0_speed_cmd = -2.8973

            franka_cmd.control_flag = 1
            franka_cmd.joint_speed_cmd[0] = joint0_speed_cmd
            franka_cmd.joint0_jerk = joint0_jerk
            franka_cmd.joint0_external_tau = joint0_external_tau
            franka_cmd.damping = action

            f1.write(str(joint0_speed) + '\n')
            f2.write(str(joint0_jerk) + '\n')
            f3.write(str(joint0_external_tau) + '\n')
            f4.write(str(action) + '\n')

        pub.publish(franka_cmd)
    else:
        inp = raw_input(str(episode_count) + " episode has been finished, continue? y/n: ")[0]
        if inp == 'y':
            episode_count += 1
            time_count = 0
            f1 = open('src/franka_ros/data/joint0_speed_' + str(episode_count) + '.txt', 'w')
            f2 = open('src/franka_ros/data/joint0_jerk_' + str(episode_count) + '.txt', 'w')
            f3 = open('src/franka_ros/data/joint0_external_tau_' + str(episode_count) + '.txt', 'w')
            f4 = open('src/franka_ros/data/damping_' + str(episode_count) + '.txt', 'w')
        else:
            print "Halting program"

def franka_control():
    rospy.init_node('fuzzy_q_learning_controller', anonymous=True)
    rospy.Subscriber("state_node", franka_state, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    franka_control()


