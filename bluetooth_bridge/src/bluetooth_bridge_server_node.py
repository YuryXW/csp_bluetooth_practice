#!/usr/bin/env python

## @package docstring
#  This package provides the bridge between Bluetooth and ROS, both ways.
#  Initially it receives "String" messages and sends "String" messages
#

import rospy
import sys
import signal
import bluetooth
import select
import time
from std_msgs.msg import String
#from ocean.depths import Cthulhu

#--------------------------------- Constants ----------------------------------#

TAG       = "Bluetooth Bridge Node:"              ## Node verbosity tag
node_name = "bluetooth_bridge"                    ## ROS node name

#------------------------------------------------------------------------------#
## Application class
#
class Application:
    ## "Is application running" flag
    is_running   = True
    ## "Is connection established" flag
    is_connected = False

    ## Input topics
    input_topic = "/bluetooth/send"             # Send a string messsage to this
                                                # topic to send it via  Bluetooth.

    ## Output topics
    output_topic = "/bluetooth/received"        # Received data from Bluetooth will
                                                # be published to this topic.
    status_topic = "/bluetooth/status"

    ## Bluetooth channel
    bt_channel = 22                             # IMPORTANT! Mae sure this is THE SAME
                                                # as was used diring
                                                # sdptool add --channel=<number> SP comand.
                                                # Also, use this command before launching
                                                # this node if you have rebooted your robot.

    ## Init function
    def __init__(self):
        # Assigning the SIGINT handler
        signal.signal(signal.SIGINT, self.sigint_handler)

        # Starting the node
        rospy.init_node(node_name, anonymous=False)

        # Getting parameters
        self.input_topic  = rospy.get_param("~send_topic", self.input_topic)
        self.output_topic = rospy.get_param("~recv_topic", self.output_topic)
        self.status_topic = rospy.get_param("~status_topic", self.status_topic)
        self.bt_channel   = rospy.get_param("~rfcomm_channel", self.bt_channel)

        print TAG, "param: input_topic  =", self.input_topic
        print TAG, "param: output_topic =", self.output_topic
        print TAG, "param: status_topic =", self.status_topic
        print TAG, "param: bt_channel   =", self.bt_channel

        # Subscribers
        self.sub        = rospy.Subscriber(self.input_topic, String, self.send_callback)

        # Publishers
        self.pub        = rospy.Publisher(self.output_topic, String, queue_size = 10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size = 10)
        time.sleep(0.5)
        self.status_pub.publish("INIT")

        while self.is_running:
            try:
                # Starting the bluetooth server
                self.server_sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
                # Listening for incoming connections
                self.server_sock.bind( ("", self.bt_channel) )
                print TAG, "Waiting for incoming connections on port %d ..." % self.bt_channel
                self.status_pub.publish("LISTENING")
                self.server_sock.listen(1)
                # Accepting incoming connection
                self.client_sock, self.address = self.server_sock.accept()
                print TAG, "Accepted connection from ", self.address
                self.status_pub.publish("CONNECTED: "+str(self.address))

                # [IMPORTANT] THIS IS HOW TO RECEIVE MESSAGE FROM BLUETOOTH
                # AND PUBLISH IT TO ROS

                # Running the loop to receive messages
                self.is_connected  = True
                while self.is_running:
                    ready = select.select([self.client_sock],[],[], 2)
                    if ready[0]:
                        data = self.client_sock.recv(1024)
                        print TAG, "Received: ", data
                        self.pub.publish(data)

            except Exception, e:
                self.is_connected = False
                self.server_sock.close()
                print TAG, "EXCEPTION:", str(e)
                self.status_pub.publish("EXCEPTION: "+str(e))
                print TAG, "RESTARTING SERVER"
                time.sleep(0.1)

    ## SIGINT Signal handler, you need this one to interrupt your node
    def sigint_handler(self, signal, frame):
            print ""
            print TAG,"Interrupt!"
            self.status_pub.publish("SIGINT")
            self.is_running = False
            print TAG,"Terminated"
            sys.exit(0)

    ## [IMPORTANT] THIS IS HOW TO SEND MESSAGES VIA BLUETOOTH
    ## Handler for the messages to be sent via bluetooth.
    def send_callback(self, message):
        if self.is_connected:
            print TAG, "Sending:", message.data
            self.client_sock.send(message.data+"\n")

#------------------------------------- Main -------------------------------------#

if __name__ == '__main__':
    print TAG,"Started"

    app = Application()

    print TAG,"Terminated"
