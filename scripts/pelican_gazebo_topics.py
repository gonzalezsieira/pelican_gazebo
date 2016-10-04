#!/usr/bin/env python

import rospy
import tf
import gazebo_msgs.msg
import geometry_msgs.msg

node_name = 'gazebo_topics'


class Publisher:

    def __init__(self, robot_name, world_name, tf_publisher, pose_publisher, vel_publisher):
        """
        Initializes the publisher topics that will process the gazebo msg
        :param tf_publisher: publisher of the transform (TFMessage)
        :param pose_publisher: publisher of the pose (PoseStamped)
        :param vel_publisher: publisher of the speed (TwistStamped)
        """
        self.tf_publisher = tf_publisher
        self.pose_publisher = pose_publisher
        self.vel_publisher = vel_publisher
        self.robot_name = robot_name
        self.world_name = world_name

    def callback_msg_model(self, msg):
        """
        Processes an input message in the form of ModelStates and publishes
        the information as messages of pose/speed/tf.
        :param msg: input message (gazebo_msgs.msg.ModelStates)
        """
        if self.robot_name in msg.name:
            # Extract info from msg
            index = msg.name.index(self.robot_name)
            pose = msg.pose[index]
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
            # Publish pose
            msg = geometry_msgs.msg.PoseStamped()
            msg.header.frame_id = self.world_name
            msg.header.stamp = timestamp
            msg.pose = pose
            self.pose_publisher.publish(msg)

            # Publish twist
            msg = geometry_msgs.msg.TwistStamped()
            msg.header.frame_id = self.world_name
            msg.header.stamp = timestamp
            msg.twist = vel
            self.vel_publisher.publish(msg)

        else:
            rospy.logwarn('Model ' + self.robot_name + ' not in gazebo ModelStates')


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
        geometry_msgs.msg.PoseStamped,
        queue_size=10
    )
    vel_publisher = rospy.Publisher(
        rospy.get_param('/' + node_name + '/publishers/velocity'),
        geometry_msgs.msg.TwistStamped,
        queue_size=10
    )
    # Get robot name
    robot_name = rospy.get_param('/' + node_name + '/model_name')
    world_name = rospy.get_param('/' + node_name + '/world_name')
    # Create structure to publish
    publishers = Publisher(robot_name, world_name, tf_publisher, pose_publisher, vel_publisher)
    # Create subscriber with this last structure
    rospy.Subscriber(
        rospy.get_param('/' + node_name + '/subscribers/gazebo_models'),
        gazebo_msgs.msg.ModelStates,
        publishers.callback_msg_model
    )
    rospy.spin()


if __name__ == '__main__':
    init_gazebo_topics()