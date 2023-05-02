#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# This following line is just to show how you can import your own custom message
# it should follow the convention: name_of_your_package.msg import name_of_your_message
from demo_python.msg import Points

# Import your custom code to implement your algorithm logic here
# for example:
from bayes_filter import BayesFilter

class DemoNode:

    def __init__(self):

        # Initialize some necessary variables here
        self.node_frequency = None
        self.sub_fake_sensor_topic = None
        self.pub_demo_topic = None
        
        # Create the bayes filter object here
        self.bayes_filter = BayesFilter()

        # Store the data received from a fake sensor
        self.fake_sensor = 0.0
        
        # Initialize the ROS node
        rospy.init_node('demo_node')
        rospy.loginfo_once('Demo Node has started')

        # Load parameters from the parameter server
        self.load_parameters()

        # Initialize the publishers and subscribers
        self.initialize_subscribers()
        self.initialize_publishers()
        
        # Initialize the timer with the corresponding interruption to work at a constant rate
        self.initialize_timer()

    def load_parameters(self):
        """
        Load the parameters from the configuration server (ROS)
        """

        # Node frequency of operation
        self.node_frequency = rospy.get_param('node_frequency', 30)
        rospy.loginfo('Node Frequency: %s', self.node_frequency)

    def initialize_subscribers(self):
        """
        Initialize the subscribers to the topics. You should subscribe to
        sensor data and odometry (if applicable) here
        """

        # Subscribe to the topic '/fake_sensor_topic'
        self.sub_fake_sensor_topic = rospy.Subscriber('/fake_sensor_topic', Float64, self.callback_fake_sensor_topic)

    def initialize_publishers(self):
        """
        You should/can initialize the publishers for the results of your algorithm here.
        """

        # Initialize the publisher to the topic '/output_topic'
        self.pub_demo_topic = rospy.Publisher('/output_topic', Odometry, queue_size=10)

    def initialize_timer(self):
        """
        Here we create a timer to trigger the callback function at a fixed rate.
        """
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.node_frequency), self.timer_callback)
        self.h_timerActivate = True

    def timer_callback(self, timer):
        """Here you should invoke methods to perform the logic computations of your algorithm.
        Note, the timer object is not used here, but it is passed as an argument to the callback by default.
        This callback is called at a fixed rate as defined in the initialization of the timer.

        At the end of the calculations of your EKF, UKF, Particle Filer, or SLAM algorithm, you should publish the results to the corresponding topics.
        """

        # Do something here at a fixed rate
        rospy.loginfo('Timer callback called at: %s', rospy.get_time())

        # Perform some logic with the sensor data received
        my_fancy_formula = self.fake_sensor + 1.0
        
        # For example, if you were implementing a bayes filter, you could do the following
        self.bayes_filter.predict(0.03, 0.5)
        # Update the model of our filter (this is just a placeholder for your algorithm)
        self.bayes_filter.update(2)

        # Create the message to publish
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'filter'
        msg.pose.pose.position.x = my_fancy_formula

        # Publish the data received to the topic '/output_topic'
        self.pub_demo_topic.publish(msg)

    def callback_fake_sensor_topic(self, msg):
        """
        Callback function for the subscriber of the topic '/demo_topic'. This function is called
        whenever a message is received by the subscriber. For example, you can receive the data from
        a sensor here and store it in a variable to be processed later or perform prediction steps
        of your algorithm here (it is up to you to decide).
        """

        # Do something with the message received
        rospy.loginfo('Received data: %s', msg.data)

        # Store the sensor message to be processed later (or process now depending on the application)
        self.fake_sensor = msg.data
        

def main():

    # Create an instance of the DemoNode class
    demo_node = DemoNode()

    # Spin to keep the script for exiting
    rospy.spin()

if __name__ == '__main__':
    main()
