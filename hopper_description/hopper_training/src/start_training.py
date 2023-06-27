#!/usr/bin/env python3

'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Moded by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import gym
import time
import numpy
import random
import time
# ROS packages required
import rclpy
from rclpy.node import Node
from hopper_training.msg import QLearnMatrix, QLearnElement, QLearnPoint
from std_msgs.msg import Float64


# import our training environment
import monoped_env


class MonopedGymNode(Node):

    def __init__(self):
        super().__init__('monoped_gym')
        publish_Q_learn = False
        # Create the Gym environment
        env = gym.make('Monoped-v0')
        self.get_logger().info('Gym environment done')
        self.reward_pub = self.create_publisher(Float64, '/monoped/reward', 1)
        self.episode_reward_pub = self.create_publisher(Float64, '/monoped/episode_reward', 1)
        if publish_Q_learn:
            self.qlearn_pub = self.create_publisher(QLearnMatrix, '/q_learn_matrix', 1)

        last_time_steps = numpy.ndarray(0)

        # Loads parameters from the ROS param server
        # Parameters are stored in a yaml file inside the config directory
        # They are loaded at runtime by the launch file
        Alpha = self.get_parameter('alpha').get_parameter_value().double_value
        Epsilon = self.get_parameter('epsilon').get_parameter_value().double_value
        Gamma = self.get_parameter('gamma').get_parameter_value().double_value
        epsilon_discount = self.get_parameter('epsilon_discount').get_parameter_value().double_value
        nepisodes = self.get_parameter('nepisodes').get_parameter_value().integer_value
        nsteps = self.get_parameter('nsteps').get_parameter_value().integer_value

        # Initialises the algorithm that we are going to use for learning
        qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                               alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
        initial_epsilon = qlearn.epsilon

        start_time = time.time()
        highest_reward = 0

        # Starts the main training loop: the one about the episodes to do
        for x in range(nepisodes):
            self.get_logger().info("STARTING Episode #" + str(x))

            cumulated_reward = 0
            cumulated_reward_msg = Float64()
            episode_reward_msg = Float64()
            done = False
            if qlearn.epsilon > 0.05:
                qlearn.epsilon *= epsilon_discount

            # Initialize the environment and get first state of the robot
            self.get_logger().info("env.reset...")
            # Now We return directly the stringified observations called state
            state = env.reset()

            self.get_logger().info("env.get_state...==>" + str(state))

            # for each episode, we test the robot for nsteps
            for i in range(nsteps):
                if publish_Q_learn:
                    # Publish in ROS topics the Qlearn data
                    Q_matrix = qlearn.get_Q_matrix()
                    qlearn_msg = self.fill_qLearn_message(Q_matrix)
                    self.qlearn_pub.publish(qlearn_msg)

                # Pick an action based on the current state
                action = qlearn.chooseAction(state)

                # Execute the action in the environment and get feedback
                self.get_logger().info("###################### Start Step...[" + str(i) + "]")
                self.get_logger().info("haa+,haa-,hfe+,hfe-,kfe+,kfe-,NoMovement >> [0,1,2,3,4,5,6]")
                self.get_logger().info("Action Space==" + str(range(env.action_space.n)))
                self.get_logger().info("Action to Perform >> " + str(action))
                nextState, reward, done, info = env.step(action)
                self.get_logger().info("END Step...")
                self.get_logger().info("Reward ==> " + str(reward))
                cumulated_reward += reward
                if highest_reward < cumulated_reward:
                    highest_reward = cumulated_reward

                self.get_logger().info("env.get_state...==>" + str(nextState))

                # Make the algorithm learn based on the results
                qlearn.learn(state, action, reward, nextState)

                # We publish the cumulated reward
                cumulated_reward_msg.data = cumulated_reward
                self.reward_pub.publish(cumulated_reward_msg)

                if not done:
                    state = nextState
                else:
                    self.get_logger().info("DONE EPISODE!")
                    last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                    break

                self.get_logger().info("###################### END Step...[" + str(i) + "]")
                # raw_input("Press_Key_to_Next_STEP...")

            m, s = divmod(int(time.time() - start_time), 60)
            h, m = divmod(m, 60)
            episode_reward_msg.data = cumulated_reward
            self.episode_reward_pub.publish(episode_reward_msg)
            self.get_logger().warn("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) +
                                   " - gamma: " + str(round(qlearn.gamma, 2)) + " - epsilon: " +
                                   str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(cumulated_reward) +
                                   "     Time: %d:%02d:%02d" % (h, m, s))
            # raw_input("Press_Key_to_Next_EPISODE>>>>>>>>>>>>>>>>>>>>>><")

        self.get_logger().info("|" + str(nepisodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" +
                               str(initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) +
                               "| PICTURE |")

        l = last_time_steps.tolist()
        l.sort()

        self.get_logger().info("Overall score: {:0.2f}".format(last_time_steps.mean()))
        self.get_logger().info("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

        env.close()

    def fill_qLearn_message(self, qlearn_dict):
        q_learn_message = QLearnMatrix()
        for q_object, q_reward in qlearn_dict.items():
            q_learn_element = QLearnElement()
            q_learn_element.qlearn_point.state_tag.data = q_object[0]
            q_learn_element.qlearn_point.action_number.data = q_object[1]
            q_learn_element.reward.data = q_reward
            q_learn_message.q_learn_matrix.append(q_learn_element)

        return q_learn_message


def main(args=None):
    rclpy.init(args=args)
    monoped_gym_node = MonopedGymNode()
    rclpy.spin(monoped_gym_node)
    monoped_gym_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
