import rospy
from std_msgs.msg import String

class Talker:
        nMinus1 = 1
        nMinus2 = 0
        def __int__(self):
            self.pub = rospy.Publisher('chatter', Float32, queue_size=10)

        def talk(self):
                
                n = Talker.nMinus1 + Talker.nMinus2  #"hello world %s" % rospy.get_time()
                Talker.nMinus2 = Talker.nMinus1
                Talker.nMinus1 = n
                rospy.loginfo(n)
                self.pub.publish(n)

if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous = True)
        t = Talker()
        rate = rospu.Rate(1) #1hz
        while not rospy.is_shutdown();
            t.talk()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass