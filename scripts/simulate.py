#!/usr/bin/env python
import rospy, sys
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_list(arg,type,s,len):
    return [ type(a) for a in arg[s:s+len] ]

# Usage
# simulate.py --parent_frame parent --child_frame child --mean 0 0 0 0 0 0 1 --stddev 0.1 0.1 0.1 0.1 0.1 0.1

argv = sys.argv
i = 0
while i < len(argv):
    opt = sys.argv[i]
    i += 1
    if opt == "--mean":
        _mean = get_list (argv, float, i, 7)
        mean = _mean[:3] + list(euler_from_quaternion(_mean[3:]))
        i += 7
    elif opt == "--stddev":
        stddev = get_list (argv, float, i, 6)
        i += 6
    elif opt == "--parent_frame":
        parent_frame = str(argv[i])
        i += 1
    elif opt == "--child_frame":
        child_frame = str(argv[i])
        i += 1
    else:
        rospy.logwarn("Argument {} not handled".format(opt))

def generateGaussian (mean, stddev):
    from random import gauss
    rand = [0,] * len(mean)
    for i,(m,s) in enumerate(zip(mean,stddev)):
        rand[i] = gauss (m, s)
    quat = quaternion_from_euler (*rand[3:])
    return Transform(Vector3(*rand[:3]), Quaternion(*quat))

rospy.init_node("simulate")

pub = rospy.Publisher("/agimus/vision/tags", TransformStamped, queue_size=10)

rate = rospy.Rate(25)

ts = TransformStamped()
ts.header.frame_id = parent_frame
ts.child_frame_id = child_frame

while not rospy.is_shutdown():
    ts.transform = generateGaussian(mean, stddev)
    ts.header.stamp = rospy.Time.now()
    ts.header.seq   += 1

    pub.publish(ts)
    rate.sleep()
