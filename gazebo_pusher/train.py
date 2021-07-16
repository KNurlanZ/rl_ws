import numpy as np
import gym
import gazebo_pusher
import qlearn

import time
import matplotlib.pyplot as plt

import random
import gym
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from gym import utils, spaces
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from gym.utils import seeding
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetLinkState

if __name__ == '__main__':
    env = gym.make('Planar5DoF-v0')
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.4, gamma=0.6, epsilon=0.99)
    highest_reward = -1
    f = open("qtable.txt", "w+")
    fa = open("actions.txt", "w+")
    result = []
    episode = []
    dx = []
    act = list()
    for x in range(5000):
        act1 = list()
        total_reward = 0
        if (x == 0):
            time.sleep(3) # to load all controllers first
        print("New episode #" + str(x))
        state = env.reset()

        if qlearn.epsilon > 0.02:
            qlearn.epsilon *= 0.99
        print(qlearn.epsilon)
        sensor1 = sensor2 = sensor3 = sensor4 = sensor5 = None
        #sensor1 = rospy.SubscribeListener('/robot_sensor1', String, callback)
        sensor1 = rospy.wait_for_message('/robot_sensor1', ContactsState)
        sensor2 = rospy.wait_for_message('/robot_sensor2', ContactsState)
        sensor3 = rospy.wait_for_message('/robot_sensor3', ContactsState)
        sensor4 = rospy.wait_for_message('/robot_sensor4', ContactsState)
        sensor5 = rospy.wait_for_message('/robot_sensor5', ContactsState)
        i=1    
        while (sensor1 is not None or sensor2 is not None or sensor3 is not None or sensor4 is not None or sensor5 is not None): 
        #for i in range(50):
            print("New step "+str(i))
            i=i+1
            action = qlearn.chooseAction(state)
            act1.append(action)
            new_state, reward, done, dex = env.step(action)
            total_reward += reward
            print("States we were "+str(state)+" action we took "+str(action)+" state we occured "+\
            str(new_state)+" reward we obtained "+str(reward)+" and total reward is "+str(total_reward))
            if highest_reward < total_reward:
                highest_reward = total_reward
                act = act1
            qlearn.learn(state,action,reward,new_state)
            if (done):
                break
            else:
                state = new_state
        episode.append(x)
        dx.append(dex)
        result.append(total_reward)
        print("Episode #"+str(x)+" has ended, total reward is "+str(total_reward))
        state = env.reset()
        
    fa.write(str(act))
    f.write(str(qlearn.q))
    f.close()
    fa.close()
    z = np.polyfit(episode, result, 1)
    p = np.poly1d(z)
    m = np.polyfit(episode, dx, 1)
    k = np.poly1d(m)
    plt.subplot(2, 1, 1)
    plt.plot(episode, result)
    plt.plot(episode, p(episode), 'r', markersize=10)
    plt.subplot(2, 1, 2)
    plt.plot(episode, dx, 'g')
    plt.plot(episode, k(episode), 'r', markersize=10)
    plt.show()    
