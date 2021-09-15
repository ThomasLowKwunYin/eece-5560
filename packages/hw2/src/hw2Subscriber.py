#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Listener:
        def __init__(self):
                rospy.Subscriber("chatter", Float32, self.callback)

        def callback(selfself,msg):
            rospy.loginfo(f"{rospy.get_caller_id()}+I heard {msg.data}")

if __name__ == '__main__':
    rospy.init_node('listener', anonymous = True)
    Listener()
    rospy.spin()