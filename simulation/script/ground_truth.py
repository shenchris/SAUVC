import rospy
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
from nav_msgs.msg import Odometry


def draw_data(_):
    global x, y, draw_x, draw_y
    draw_x.append(x)
    draw_y.append(y)
    plt.cla()
    plt.scatter(draw_x, draw_y)
    plt.plot(draw_x, draw_y)


def set_data(data):
    global draw_x, draw_y, x, y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    print(f'x: {x} y: {y}')


def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/vec6/sim_ground_truth", Odometry, set_data)
    anima = animation.FuncAnimation(plt.gcf(), draw_data, interval=10)
    plt.show()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    global x, y, draw_x, draw_y
    draw_x = []
    draw_y = []
    listener()
