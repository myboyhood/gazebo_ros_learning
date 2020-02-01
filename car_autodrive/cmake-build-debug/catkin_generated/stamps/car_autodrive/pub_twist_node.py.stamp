import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Autodrive:
    def __init__(self):
        self.odom = Odometry()
        self.origin_twist = Twist()
        self.update_twist = Twist()
        self.error_y = 0
        self.error_y_ctrl = 0
        self.err_y = 0

    def odom_cb(self, msg):
        self.odom = msg

    def cmd_vel_origin_cb(self, msg):
        self.origin_twist = msg

    def error_diedlock(self, err_y):
        if abs(err_y) < 0.01:
            self.err_y = 0.
        else:
            self.err_y = err_y
        return self.err_y

    def car_autodrive(self):

        rospy.init_node('pub_twist_node')

        rate = rospy.Rate(10)
        rate.sleep()

        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber('cmd_vel_origin', Twist, self.cmd_vel_origin_cb)
        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        POSI_P = 1

        while not rospy.is_shutdown():

            self.update_twist.linear.x = self.origin_twist.linear.x

            self.error_y = 0 - self.odom.pose.pose.position.y
            self.error_y = self.error_diedlock(self.error_y)
            self.error_y_ctrl = POSI_P * self.error_y
            self.update_twist.angular.z = self.error_y_ctrl
            twist_pub.publish(self.update_twist)

            print("linear_vel: " , self.update_twist.linear.x, "\t angular_vel %s", self.update_twist.angular.z)


if __name__=="__main__":
    try:
        car = Autodrive()
        car.car_autodrive()
    except rospy.ROSInterruptException:
        pass
