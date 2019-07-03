import rospy
import tf
import tf2_ros
import thread
import pickle
import math
import numpy as np


rospy.init_node('saveTf')

transfo = tf.TransformerROS()
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf)
listener_acq = tf.TransformListener()


def fitSphere(points):
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    z = np.array([p[2] for p in points])
    
    # (x - x_0)^2 + (y - y_0)^2 + (z - z_0)^2 = r^2
    # x^2 + y^2 + z^2 = 2xx_0 + 2yy_0 + 2zz_0 - x_0^2 - y_0^2 - z_0^2 + r^2
    # AX = b
    # With b = [x^2 + y^2 + z^2]...for all (x, y, z)
    # A = [2x 2y 2z 1]...for all (x, y, z)
    # X = [x_0 y_0 z_0 (-x_0^2 - y_0^2 - z_0^2 + r^2)]^T only 4 components
    
    b = np.zeros((len(x),1))
    b[:,0] = (x*x) + (y*y) + (z*z)
    
    A = np.zeros((len(x),4))
    A[:,0] = x*2
    A[:,1] = y*2
    A[:,2] = z*2
    A[:,3] = 1

    X, _, _, _ = np.linalg.lstsq(A,b)

    # Find the radius
    r = X[3] + (X[0]*X[0] + X[1]*X[1] + X[2]*X[2])
    r = math.sqrt(r)

    return X[0][0], X[1][0], X[2][0], r

def fitPlane(points):
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    z = np.array([p[2] for p in points])

    # ax + by + cz + d = 0
    # TODO: Case if a == 0
    # else - x = b'y + c'z + d'
    # AX = f
    # f = [-x]...for all (x, y, z)
    # A = [y z 1]...for all (x, y, z)
    # X = [b' c' d']^T only 3 components

    f = np.zeros((len(x),1))
    f[:,0] = -x

    A = np.zeros((len(x),3))
    A[:,0] = y
    A[:,1] = z
    A[:,2] = 1

    X, _, _, _ = np.linalg.lstsq(A,f)

    return 1., X[0,0], X[1,0], X[2,0]

def distPointPlane(point, plane):
    num = math.fabs(point[0]*plane[0] + point[1]*plane[1] + point[2]*plane[2] + plane[3])
    den = math.sqrt(plane[0]*plane[0] + plane[1]*plane[1] + plane[2]*plane[2])
    
    return num / den

def acquisition(list_id_tags):
    def stopWhile(var):
        # Hack to stop a while loop by typing smth
        raw_input()
        var.append(True)
    var = [] 
    thread.start_new_thread(stopWhile, (var,))
    
    points = {id_tag:[] for id_tag in list_id_tags}
    rate = rospy.Rate(30)
    print('Recording until you type in some text and press enter...')
    while not var:
        for id_tag in list_id_tags:
            try:
                (trans, rot) = listener_acq.lookupTransform('/base_link', '/{}'.format(id_tag), rospy.Time(0))
                points[id_tag].append((trans, rot))
            except:
                continue
        rate.sleep()
    
    print('End of recording')
    return points

def keepTrans(points):
    if isinstance(points, dict):
        for key in points.keys():
            points[key] = keepTrans(points[key])
    else:
        for i in range(len(points)):
            points[i] = points[i][0]
    
    return points


def rigid_transform_3D(A, B):
    ''' Input: expects Nx3 matrix of points
     Returns R,t
     R = 3x3 rotation matrix
     t = 3x1 column vector'''

    A = np.matrix(A)
    B = np.matrix(B)
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.transpose(AA).dot(BB)

    U, S, Vt = np.linalg.svd(H)

    R = (Vt.T).dot(U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = (Vt.T).dot(U.T)

    t = -R.dot(centroid_A.T) + centroid_B.T

    print t

    return R, t

def meanTranslationRotation(pose):
    t = np.stack([p[0] for p in pose], axis = 0)
    q = np.stack([p[1] for p in pose], axis = 0)
    
    eig_val, eig_vec = np.linalg.eig(q.T.dot(q))
    
    return np.mean(t, axis = 0), eig_vec[:,list(eig_val).index(max(list(eig_val)))]

def frameOrigin():
    frame_origin = np.zeros((4, 3))
    frame_origin[1, 0] = frame_origin[2, 1] = frame_origin[3, 2] = 1.
    return frame_origin

def getVecArm_7_link(side):
    tr = buf.lookup_transform('base_link', 'arm_'+side+'_7_link', rospy.Time(0))
    t = tr.transform.translation
    t = [t.x, t.y, t.z]
    q = tr.transform.rotation
    q = [q.x, q.y, q.z, q.w]
    vec = dict()
    vec['x'] = np.array((transfo.fromTranslationRotation(t, q) * np.matrix([1., 0., 0., 1.]).T)[0:3].T)[0]
    vec['y'] = np.array((transfo.fromTranslationRotation(t, q) * np.matrix([0., 1., 0., 1.]).T)[0:3].T)[0]
    vec['z'] = np.array((transfo.fromTranslationRotation(t, q) * np.matrix([0., 0., 1., 1.]).T)[0:3].T)[0]
    return vec


def acquireAxisForTags(tags):
    ''' Tags = {tag_frame: {}, tag2_frame: {}...}'''
    for tag in tags.keys():
        raw_input('Tracking tag {}, put it in front of the camera'.format(tag))
        
        print('Acquisition of the tag\'s pose')
        tags[tag]['points'] = acquisition([tag])[tag]
        tags[tag]['bMt'] = np.matrix(transfo.fromTranslationRotation(*meanTranslationRotation(tags[tag]['points'])))
        
        tags[tag]['axis'] = dict()
        tags[tag]['vec'] = dict()

        for axis, joint in {'x': 6, 'y': 7, 'z': 5}.iteritems():
            print('Move axis {} (joint {}) bound to bound then back to its initial value'.format(axis, joint))
            tags[tag]['axis'][axis] = keepTrans(acquisition([tag]))[tag]
            tags[tag]['vec'][axis] = np.array(fitPlane(tags[tag]['axis'][axis])[0:3])
            tags[tag]['vec'][axis] /= np.linalg.norm(tags[tag]['vec'][axis])
            if tags[tag]['vec'][axis].dot(getVecArm_7_link(side)[axis]) < 0.:
                tags[tag]['vec'][axis] *= -1.


        tags[tag]['all_axis'] = tags[tag]['axis']['x'] + tags[tag]['axis']['y']+ tags[tag]['axis']['z']
    return tags

def calibrateJointForTags(tags):
    ''' Tags = {tag_frame: {}, tag2_frame: {}...}'''
    tMj = dict()

    sphere_centre = np.sum(np.stack([np.array(fitSphere(tag['all_axis'])[0:3]) * len(tag['all_axis']) for tag in tags.itervalues()], axis=0), axis=0) / sum([len(tag['all_axis']) for tag in tags.itervalues() ])
    
    for tag in tags.keys():
        tags[tag]['frame'] = np.matrix([sphere_centre, 
                                        sphere_centre + tags[tag]['vec']['x'], 
                                        sphere_centre + tags[tag]['vec']['y'],
                                        sphere_centre + tags[tag]['vec']['z']])
        print(tags[tag]['frame'])
        tags[tag]['R'], tags[tag]['T'] = rigid_transform_3D(frameOrigin(), tags[tag]['frame'])
        if np.linalg.norm(tags[tag]['T'].T - sphere_centre) > 2.e-1:
            tags[tag]['frame'][3:4, 0:3] -= tags[tag]['vec']['z'] * 2.
            tags[tag]['R'], tags[tag]['T'] = rigid_transform_3D(frameOrigin(), tags[tag]['frame'])
        tags[tag]['bMj'] = np.zeros((4, 4))
        tags[tag]['bMj'][3, 3] = 1.
        tags[tag]['bMj'][0:3, 3:4] = tags[tag]['T']
        tags[tag]['bMj'][0:3, 0:3] = tags[tag]['R'] 

        tags[tag]['tMj'] = tags[tag]['bMt'].I.dot(tags[tag]['bMj'])
        tMj[tag] = tags[tag]['tMj']
        # At this time, we have bMj (in T and R) and bMt (in tags) so we can compute tMj and save it!
    
    return tMj



tags = {'left':  {100: {}, 101: {}, 102: {}, 103: {}},
        'right': {104: {}, 105: {}, 106: {}, 107: {}}}
for side in ['left', 'right']:
    with open(side+'_hand_tags_calib.pickle', 'w') as f:
        pickle.dump(calibrateJointForTags(acquireAxisForTags(tags[side])), f)
