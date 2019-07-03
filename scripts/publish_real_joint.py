# TODO: When a marker is seen, publish transform from joint to real pose of joint

import pickle
import rospy
import tf 
import tf2_ros
import numpy
from geometry_msgs.msg import TransformStamped


rospy.init_node('hand_republisher')


class RealLink7Publisher:
    def __init__(self, side):
        self.side = side
        self.last_timestamp_used = 0.
        self.jMt_calib_transformations = pickle.load(open('tMj_'+self.side+'_hand.pickle', 'r'))
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.tf_transformer = tf.TransformerROS()
        rospy.Subscriber('/agimus/vision/tags', TransformStamped, self.republishJointPose)

    def republishJointPose(self, msg):
        # Frame not handle by this publisher
        if not self.jMt_calib_transformations.has_key(msg.child_frame_id):
            return

        time_between_publish = 0.1
        current_timestamp = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1.e9
        if current_timestamp - self.last_timestamp_used < time_between_publish:
            return
        self.last_timestamp_used = current_timestamp
        translation = (msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
        rotation = (msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w)
        jMt_mes = self.tf_transformer.fromTranslationRotation(translation, rotation)
        jMj_mes = jMt_mes.dot(self.jMt_calib_transformations[msg.child_frame_id])

        tr = TransformStamped()

        tr.header.stamp = rospy.Time.now()
        tr.header.frame_id = 'arm_'+self.side+'_7_link'
        tr.child_frame_id = 'arm_'+self.side+'_7_link_est'
        t = tf.transformations.translation_from_matrix(jMj_mes)
        tr.transform.translation.x = t[0]
        tr.transform.translation.y = t[1]
        tr.transform.translation.z = t[2]
        q = tf.transformations.quaternion_from_matrix(jMj_mes) 
        tr.transform.rotation.x = q[0]
        tr.transform.rotation.y = q[1]
        tr.transform.rotation.z = q[2]
        tr.transform.rotation.w = q[3]

        (self.tf_pub).sendTransform(tr)


left = RealLink7Publisher('left')
right = RealLink7Publisher('right')
rospy.spin()



    

