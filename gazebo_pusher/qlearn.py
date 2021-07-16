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

class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, value):
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))            
        '''
        oldv = self.q.get((state, action), None)
        if oldv is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)
    
    def chooseAction(self, state, return_q=False):
        #def callback(data):
        #    rospy.loginfo("I heard %s",data.data)
        #global string
        #rospy.init_node('/robot_sensor1')
        #stringer = rospy.wait_for_message('/robot_sensor1', ContactsState)
        #print (stringer)
        #sensor1 = sensor2 = sensor3 = sensor4 = sensor5 = None
        #sensor1 = rospy.SubscribeListener('/robot_sensor1', String, callback)
        #sensor1 = rospy.wait_for_message('/robot_sensor1', ContactsState)
        #sensor2 = rospy.wait_for_message('/robot_sensor2', ContactsState)
        #sensor3 = rospy.wait_for_message('/robot_sensor3', ContactsState)
        #sensor4 = rospy.wait_for_message('/robot_sensor4', ContactsState)
        #sensor5 = rospy.wait_for_message('/robot_sensor5', ContactsState)    
        #while (sensor1 is not None or sensor2 is not None or sensor3 is not None or sensor4 is not None or sensor5 is not None):
            
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)
        if random.random() < self.epsilon:
            #minQ = min(q); mag = max(abs(minQ), abs(maxQ))
            minQ = min(q); mag = max(abs(minQ), abs(maxQ))/2
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + random.random() * mag - .5 * mag for i in range(len(self.actions))] 
            maxQ = max(q)

        count = q.count(maxQ)
        # In case there're several state-action max values 
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]        
        if return_q: # if they want it, give it!
            return action, q
        return action

    def learn(self, state1, action1, reward, state2):
        maxqnew = max([self.getQ(state2, a) for a in self.actions])
        self.learnQ(state1, action1, reward, reward + self.gamma*maxqnew)