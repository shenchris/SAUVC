import rospy
from sensor_msgs.msg import JointState


def talker():
    state_pub = rospy.Publisher('/vec6/thruster_command', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        effort_ = JointState()
        effort_.name = ["f_port", "f_star", "m_port", "m_star", "b_port", "b_star"]  # port = left, star = right

        command = input()
        while True:
            if command == "w":
                effort_.effort = [100, 100, 0, 0, 0, 0]
                print("oo")
            elif command == "s":
                effort_.effort = [-100, -100, 0, 0, 0, 0]
            elif command == "a":
                effort_.effort = [-100, 100, 0, 0, 100, -100]
            elif command == "d":
                effort_.effort = [100, -100, 0, 0, -100, 100]
            elif command == "f":
                effort_.effort = [0, 0, 0, 0, 0, 0]
            state_pub.publish(effort_)
            command = input()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
