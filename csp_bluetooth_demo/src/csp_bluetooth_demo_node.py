#!/usr/bin/env python

## @package docstring
#  This is the typical ROS template node for python.
#

import rospy
import sys
import signal
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import BoolStamped
from std_msgs.msg import Header

#--------------------------------- Constants ----------------------------------#

TAG       = "CSP Bluetooth Demo:"               ## Node verbosity tag
node_name = "bluetooth_demo_node"               ## ROS node name

#------------------------------------------------------------------------------#
## Application class
#
class Application:
    # Current speed and omega
    v = 0.0
    omega = 0.0
    v_const = 0

    ## Init function
    #  @param self The object pointer.
    def __init__(self):
        # Assigning the SIGINT handler
        signal.signal(signal.SIGINT, self.sigint_handler)

        # Starting the node
        rospy.init_node(node_name, anonymous=True)

        # Subscribing to bluetooth_bridge output
        rospy.Subscriber("/bluetooth/received", String, self.message_handler)
        rospy.Subscriber("stop_line_filter_node/at_stop_line", BoolStamped, self.stop_line_handler)

        # Publishers
        self.cmd_pub = rospy.Publisher("joy_mapper_node/car_cmd", Twist2DStamped, queue_size = 10)
        self.bt_pub = rospy.Publisher("/bluetooth/send", String, queue_size = 10)

        # Run the node in eternal loop
        rospy.spin()

    ## SIGINT Signal handler
    #  @param self The object pointer.
    #  @param signal The signal itself.
    #  @param frame The timeframe signal occured.
    def sigint_handler(self, signal, frame):
            print ""
            print TAG,"Interrupt!"
            print TAG,"Terminated"
            sys.exit(0)

    ## Handling messages obtained from bluetooth_bridge
    def message_handler(self, message):
        s = message.data
        msgs = s.split(";")

        for msg in msgs:
            # Ordering robot to GO - move forward at constant speed
            if (msg=="GO"):
                print msg
                self.v_const = 0.4
            # Ordering robot to stop
            elif (msg=="STOP"):
                print msg
                self.v_const = 0.0
                self.v = 0.0
            else:
                # Controlling robot with Roll and Pitch values from accelerometer
                c_msg = msg.split(":")
                # Try to invert these values
                if c_msg[0]=="Roll":  # Roll controls omega
                    self.omega = (float(c_msg[1])/90.0)*8.0
                if c_msg[0]=="Pitch": # Pitch controls velocity
                    self.v = (float(c_msg[1])/90.0)

        # Let's send a message now via bluetooth_bridge
        h = Header()
        h.stamp = rospy.Time.now()

        topic_message = Twist2DStamped()
        topic_message.v = self.v+self.v_const  #Velocity is a sum of constant speed (in case of GO)
                                               #and a variable speed from accelerometer.
        topic_message.omega = self.omega
        if topic_message.v > 1.0:              #Do not let the velocity exceed -1.0..1.0 range.
            topic_message.v = 1.0
        if topic_message.v < -1.0:
            topic_message.v = -1.0

        self.cmd_pub.publish(topic_message)   #Publish the topic to bluetooth_bridge

        # Handling the Stop Line detection
    def stop_line_handler(self, data):
        print "STOP LINE DETECTED!"
        self.bt_pub.publish("VIBRATE")

#------------------------------------- Main -----------------------------------#

if __name__ == '__main__':
    print TAG,"Started"

    app = Application()

    print TAG,"Terminated"
