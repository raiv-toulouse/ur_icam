#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy


def main():
    rospy.init_node('send_joints')
    rospy.loginfo("###########################################################")
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)
    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']
    rate = rospy.Rate(1)
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()
    pts.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    pts.time_from_start = rospy.Duration(1.0)
    # Set the points to the trajectory
    traj.points = []
    traj.points.append(pts)
    # Publish the message
    pub.publish(traj)
    rospy.loginfo("-------------------------------------------------------------")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
