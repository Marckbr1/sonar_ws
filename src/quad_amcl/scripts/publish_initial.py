#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node('publish_initial_pose', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # esperar que el publisher se conecte

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    # Ajusta estos valores según el punto dentro del mapa que quieres usar:
    msg.pose.pose.position.x = 395232.3  # ejemplo
    msg.pose.pose.position.y = 6456211.1  # ejemplo
    msg.pose.pose.position.z = 0.0

    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    # Covarianza, puedes dejar todo 0 si no tienes mejor estimación:
    msg.pose.covariance = [0.0]*36

    rospy.loginfo("Publishing initial pose...")
    pub.publish(msg)

    rospy.spin()

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass

