#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

#from TCP_CMD.msg import tcpCMD



CMD = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

def callback(data):

    global CMD
    rospy.loginfo("%f, %f", data.axes[1], data.axes[2])
    # left_vertical = data.axes[1]
    # right_horizontal = data.axes[2]
    CMD[0] = data.axes[3] * 1000
    CMD[1] = data.axes[1] * 1000
    if data.buttons[0] == 1:
        CMD[2] = 100
    else:
        CMD[2] = 0

def joyTranslate():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    #pub_left = rospy.Publisher('pedal_pos', Float32, queue_size = 10)
    #pub_right = rospy.Publisher('steering_pos', Float32, queue_size = 10)
    # pub_st = rospy.Publisher('steering_cmd', Float32, queue_size = 10)
    # pub_pd = rospy.Publisher('speed_setpoint', Float32, queue_size = 10)
    # pub_ec = rospy.Publisher('engine_cut', Bool, queue_size = 10)
    rospy.init_node('joyTranslate', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber("joy", Joy, callback)
    pub = rospy.Publisher("ecat_ms_out", Int16MultiArray, queue_size = 10)
    #msg = tcpCMD()
    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    while not rospy.is_shutdown():
        #pub_left.publish(left_vertical)
        #pub_right.publish(right_horizontal)
        #msg.pedal_percent = left_vertical
        # pub_pd.publish(left_vertical)
        # pub_st.publish(right_horizontal)
        # #msg.steering_percent = right_horizontal
        # pub_ec.publish(engine_cut)
        pub.publish(data=CMD)
        rate.sleep()

if __name__ == '__main__':
    joyTranslate()
