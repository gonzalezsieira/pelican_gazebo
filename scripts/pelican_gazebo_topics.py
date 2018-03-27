#!/usr/bin/env python

import rospy
import math
import numpy
import tf
import std_msgs.msg
import gazebo_msgs.msg
import geometry_msgs.msg
import asctec_hl_comm.msg
import tf.transformations

node_name = 'gazebo_topics'


class Publisher:

    def __init__(self, robot_name, world_name, tf_publisher, pose_publisher, vel_publisher, cmd_publisher, ekf_publisher, cmd_mav_publisher):
        """
        Initializes the publisher topics that will process the gazebo msg
        :param tf_publisher: publisher of the transform (TFMessage)
        :param pose_publisher: publisher of the pose (PoseStamped)
        :param vel_publisher: publisher of the speed (TwistStamped)
        """
        self.tf_publisher = tf_publisher
        self.pose_publisher = pose_publisher
        self.vel_publisher = vel_publisher
        self.cmd_publisher = cmd_publisher
        self.robot_name = robot_name
        self.world_name = world_name
        self.last_vel = None
        self.ekf_publisher = ekf_publisher
        self.cmd_mav_publisher = cmd_mav_publisher

    def callback_msg_model(self, msg):
        """
        Processes an input message in the form of ModelStates and publishes
        the information as messages of pose/speed/tf.
        :param msg: input message (gazebo_msgs.msg.ModelStates)
        """
        if self.robot_name in msg.name:
            # Extract info from msg
            index = msg.name.index(self.robot_name)
            # will be geometry_msgs.Pose
            pose = msg.pose[index]
            # will be geometry_msgs.Twist
            vel = msg.twist[index]
            # Current timestamp
            timestamp = rospy.Time.now()
            # Publish tf
            self.tf_publisher.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                timestamp,
                self.robot_name,
                self.world_name
            )

            # Publish twist (geometry_msgs.TwistStamped)
            msg = geometry_msgs.msg.TwistStamped()
            msg.header.frame_id = self.world_name
            msg.header.stamp = timestamp
            msg.twist = vel
            self.vel_publisher.publish(msg)

            # Publish PoseWithCovarianceStamped
            msg = geometry_msgs.msg.PoseWithCovarianceStamped()
            msg.header.frame_id = self.world_name
            msg.header.stamp = timestamp
            msg.pose.pose = pose

            self.pose_publisher.publish(msg)

            # convert to euler angles
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

            # Set state
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            # Set velocities
            vel_x = vel.linear.x
            vel_y = vel.linear.y
            vel_z = vel.linear.z

            # Angular velocities
            vel_roll = vel.angular.x
            vel_pitch = vel.angular.y
            vel_yaw = vel.angular.z

            # Rotate angular velocities
            vel_x, vel_y, vel_z = (matrix_rotation(-roll, -pitch, -yaw) * numpy.matrix([vel_x, vel_y, vel_z]).transpose()).getA1()

            # Publish filter state (std_msgs.Float32MultiArray)
            if self.last_vel is not None:

                # Calculate accelerations
                diff_t = (timestamp - self.last_vel[0]).to_sec()
                acc_x = (vel_x - self.last_vel[1][0]) / diff_t
                acc_y = (vel_y - self.last_vel[1][1]) / diff_t
                acc_z = (vel_z - self.last_vel[1][2]) / diff_t

                # Build msg
                msg = std_msgs.msg.Float32MultiArray()
                msg.data = [x, y, z, roll, pitch, yaw, vel_x, vel_y, vel_z, vel_roll, vel_pitch, vel_yaw, acc_x, acc_y, acc_z]
                self.ekf_publisher.publish(msg)

            self.last_vel = (timestamp, [vel_x, vel_y, vel_z])

        else:
            rospy.logwarn('Model ' + self.robot_name + ' not in gazebo ModelStates, names are: ' + str(msg.name))

    def callback_cmd(self, msg):
        # Convert to asctec mav framework format (asctec_hl_comm.mav_ctrl)
        msg_mav = asctec_hl_comm.msg.mav_ctrl()
        msg_mav.x = msg.linear.x
        msg_mav.y = msg.linear.y
        msg_mav.z = msg.linear.z
        msg_mav.yaw = msg.angular.z
        # Complete required parameters
        msg_mav.v_max_xy = 4.0
        msg_mav.v_max_z = 4.0
        msg_mav.type = asctec_hl_comm.msg.mav_ctrl.velocity_body

        # Send messages
        self.cmd_publisher.publish(msg)
        self.cmd_mav_publisher.publish(msg_mav)


def matrix_rotation(roll, pitch, yaw):
    """
    Rotation matrix is obtained by multiplying R(yaw) * R(pitch) * R(roll). This returns the
    result of the multiply operation as stated in http://planning.cs.uiuc.edu/node102.html.
    :param roll: rotation angle in the X axis
    :param pitch: rotation angle in the Y axis
    :param yaw: rotation angle in the Z axis
    :return: rotation matrix in 3D
    """
    return numpy.matrix([
        [math.cos(yaw) * math.cos(pitch), math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll), math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)],
        [math.sin(yaw) * math.cos(pitch), math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll), math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
        [-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll)]
    ])

def init_gazebo_topics():
    """
    Initializes the topics to listen/publish messages.
    """
    rospy.init_node(node_name)
    # Create Subscribers
    rospy.get_param('')
    # Create Publishers
    tf_publisher = tf.TransformBroadcaster()
    pose_publisher = rospy.Publisher(
        rospy.get_param('/' + node_name + '/publishers/pose'),
        geometry_msgs.msg.PoseWithCovarianceStamped,
        queue_size=10
    )
    cmd_publisher = rospy.Publisher(
        rospy.get_param('/' + node_name + '/publishers/cmd'),
        geometry_msgs.msg.Twist,
        queue_size=10
    )
    vel_publisher = rospy.Publisher(
        rospy.get_param('/' + node_name + '/publishers/velocity'),
        geometry_msgs.msg.TwistStamped,
        queue_size=10
    )
    cmd_mav_publisher = rospy.Publisher(
        rospy.get_param('/' + node_name + '/publishers/cmd_mav'),
        asctec_hl_comm.msg.mav_ctrl,
        queue_size=10
    )
    ekf_publisher = rospy.Publisher(
        rospy.get_param('/' + node_name + '/publishers/ekf'),
        std_msgs.msg.Float32MultiArray,
        queue_size=10
    )
    # Get robot name
    robot_name = rospy.get_param('/' + node_name + '/model_name')
    world_name = rospy.get_param('/' + node_name + '/world_name')
    # Create structure to publish
    publishers = Publisher(robot_name, world_name, tf_publisher, pose_publisher, vel_publisher, cmd_publisher, ekf_publisher, cmd_mav_publisher)
    # Create subscriber with this last structure
    rospy.Subscriber(
        rospy.get_param('/' + node_name + '/subscribers/gazebo_models'),
        gazebo_msgs.msg.ModelStates,
        publishers.callback_msg_model
    )
    rospy.Subscriber(
        rospy.get_param('/' + node_name + '/subscribers/cmd'),
        geometry_msgs.msg.Twist,
        publishers.callback_cmd
    )
    rospy.spin()


if __name__ == '__main__':
    init_gazebo_topics()