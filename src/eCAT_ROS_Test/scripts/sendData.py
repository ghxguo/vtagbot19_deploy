import rospy
from std_msgs.msg import Int16MultiArray



if __name__=="__main__":
	rospy.init_node("sendData", anonymous = True)
	pub = rospy.Publisher("ecat_ms_out", Int16MultiArray, queue_size = 10)
	k = 0
	while not rospy.is_shutdown():
		someData = [-1000,500,2,3,4,5,6,7,8,9,10,11]
		pub.publish(Int16MultiArray(data=someData))
