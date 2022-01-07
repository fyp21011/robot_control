#! /usr/bin/env python
from __future__ import print_function
from pdb import set_trace as breakpoint
import numpy as np
from numpy import linalg
from scipy.spatial.transform import Rotation as R
import ros_numpy as rosnp
import rospy
import tf2_ros as tf2
from tf import transformations as ts
import geometry_msgs.msg
import actionlib
from robot_control.msg import ControlAction, ControlFeedback, ControlResult
from ur_move import MoveGroupPythonInteface
import helper

def recover_homogenous_affine_transformation(p, p_prime):
    '''
    Find the unique homogeneous affine transformation that
    maps a set of 3 points to another set of 3 points in 3D
    space:

        p_prime == np.dot(p, R) + t

    where `R` is an unknown rotation matrix, `t` is an unknown
    translation vector, and `p` and `p_prime` are the original
    and transformed set of points stored as row vectors:

        p       = np.array((p1,       p2,       p3))
        p_prime = np.array((p1_prime, p2_prime, p3_prime))

    The result of this function is an augmented 4-by-4
    matrix `A` that represents this affine transformation:

        np.column_stack((p_prime, (1, 1, 1))) == \
            np.dot(np.column_stack((p, (1, 1, 1))), A)

    Source: https://math.stackexchange.com/a/222170 (robjohn)
    
    >>> recover_homogenous_affine_transformation(
        np.array(((1.0,1.0,1.0),
                  (1.0,2.0,1.0),
                  (1.0,1.0,2.0))),
        np.array(((2.4142135623730940, 5.732050807568877, 0.7320508075688767),
                  (2.7677669529663684, 6.665063509461097, 0.6650635094610956),
                  (2.7677669529663675, 5.665063509461096, 1.6650635094610962))))
    array([[ 0.8660254 , -0.35355339, -0.35355339,  0.        ],
        [ 0.35355339,  0.9330127 , -0.0669873 ,  0.        ],
        [ 0.35355339, -0.0669873 ,  0.9330127 ,  0.        ],
        [ 0.84108138,  5.21957879,  0.21957879,  1.        ]])
    '''

    # construct intermediate matrix
    Q       = p[1:]       - p[0]
    Q_prime = p_prime[1:] - p_prime[0]

    # calculate rotation matrix
    R = np.dot(np.linalg.inv(np.row_stack((Q, np.cross(*Q)))),
               np.row_stack((Q_prime, np.cross(*Q_prime))))

    # calculate translation vector
    t = p_prime[0] - np.dot(p[0], R)

    # calculate affine transformation matrix
    return np.column_stack((np.row_stack((R, t)), (0, 0, 0, 1)))


# def rotation_matrix_from_vectors(vec1, vec2):
#     """ Find the rotation matrix that aligns vec1 to vec2
#     :param vec1: A 3d "source" vector
#     :param vec2: A 3d "destination" vector
#     :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
#     """
#     a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
#     v = np.cross(a, b)
#     c = np.dot(a, b)
#     s = np.linalg.norm(v)
#     kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
#     rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
#     return rotation_matrix

class Controller():
    def __init__(self, name, calib=True):
        self._ur_controller = MoveGroupPythonInteface(sim=False)
        print(self._ur_controller.get_robot_current_pose())
        # self._ur_controller.freedrive_start()
        if calib:
            self.calibrate()
        else:
            self.use_calib_data()
            
        self._tfBuffer = tf2.Buffer()
        self._listener = tf2.TransformListener(self._tfBuffer)
            

class ControllerActionServer():
    _calib_filename = 'calib_data.npz'
    _feedback = ControlFeedback()
    _result = ControlResult()
    
    def __init__(self, name, calib=True):
        self._action_name = name
        self._ur_controller = MoveGroupPythonInteface(sim=False)
        print(self._ur_controller.get_robot_current_pose())
        # self._ur_controller.freedrive_start()
        if calib:
            self.calibrate()
        else:
            self.use_calib_data()
        
        self._tfBuffer = tf2.Buffer()
        self._listener = tf2.TransformListener(self._tfBuffer)
        self.align_center()
        
        self._server = actionlib.SimpleActionServer('robot_control', ControlAction, self.execute, auto_start=False)
        self._server.start()
    
    def use_calib_data(self):
        data = np.load(self._calib_filename)
        self.orientation = data['orientation']
                
        world2base_trans = data['trans']
        world2base_rot = data['rot']
        self.send_world2base_tf(world2base_trans, world2base_rot)
        
    
    def calibrate(self):
        raw_input('move robot to the center of origin in world frame')
        pose = self._ur_controller.get_robot_current_pose()
        self.orientation = pose[3:]
        o_pos = pose[:3]
        
        raw_input('move robot to the marked point on x-axis in world frame')
        x_pos = self._ur_controller.get_robot_current_pose()[:3]
        
        # translation
        world2base_trans = o_pos
        # world2base_trans[2] += 0.05 # safety margin
        
        # rotation around z axis
        world_xaxis_in_base = (x_pos - o_pos)
        world_xaxis_in_base[2] = 0.
        world_xaxis_in_base /= np.sqrt(np.sum(world_xaxis_in_base**2))
        world2base_theta = np.arctan2(world_xaxis_in_base[1], world_xaxis_in_base[0])
        r = R.from_euler('z', world2base_theta) 
        world2base_rot = r.as_quat() # np.array([0., 0., np.sin(theta/2), np.cos(theta/2)]) #x,y,z,w
        
        np.savez(self._calib_filename, orientation=self.orientation, o_pos=o_pos, x_pos=x_pos, trans=world2base_trans, rot=world2base_rot)
        
        self.send_world2base_tf(world2base_trans, world2base_rot)
        
        
    def send_world2base_tf(self, trans, rot):
        b = tf2.StaticTransformBroadcaster()
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'world'
        m.child_frame_id = 'base_link'
        m.transform.translation = rosnp.msgify(geometry_msgs.msg.Vector3, trans)
        m.transform.rotation = rosnp.msgify(geometry_msgs.msg.Quaternion, rot)
        b.sendTransform(m)
    
    def calc_pos_world2base(self, world_pos):
        """ Tranform 3*1 position vector from `world` to `base_link` coordinate system
        
        Args:
            world_pos (array_like): position [x,y,z] in world coordinate system

        Returns:
            [numpy.ndarray]: position [x,y,z] in base_link coordinate system
        """
        world2base_tf = self._tfBuffer.lookup_transform('world', 'base_link', rospy.Time(), timeout=rospy.Duration(4.0))
        trans = rosnp.numpify(world2base_tf.transform.translation)
        quat = rosnp.numpify(world2base_tf.transform.rotation)
        r = R.from_quat(quat)
        base_pos = r.apply(world_pos) +trans
        return base_pos
    
    def calc_pos_base2world(self, base_pos):
        """ Tranform 3*1 position vector from `base_link` to `world` coordinate system
        
        Args:
            base_pos (array_like): position [x,y,z] in base_link coordinate system

        Returns:
            [numpy.ndarray]: position [x,y,z] in world coordinate system
        """
        world2base_tf = self._tfBuffer.lookup_transform('base_link', 'world', rospy.Time(), timeout=rospy.Duration(4.0))
        trans = rosnp.numpify(world2base_tf.transform.translation)
        quat = rosnp.numpify(world2base_tf.transform.rotation)
        r = R.from_quat(quat)
        world_pos = r.apply(base_pos) + trans
        return world_pos
    
    def move_to_base_pos(self, base_pos):
        pose = np.concatenate((base_pos, self.orientation))
        plan, fraction = self._ur_controller.plan_cartesian_path(helper.gen_pose(*pose))
        self._ur_controller.display_trajectory(plan)
        # raw_input('click enter to align robot to center')
        self._ur_controller.go_cartesian_path(helper.gen_pose(*pose))
    
    def move_to_world_pos(self, world_pos):
        """Clip the target position within the world boundary, and move robot to the position

        Args:
            world_pos (array_like): 3*1 array of target world position
        """
        world_pos = np.concatenate((np.clip(world_pos[:2], -0.2, 0.2), np.clip([world_pos[2]], 0., 0.4)))
        base_pos = self.calc_pos_world2base(world_pos)
        self.move_to_base_pos(base_pos)
    
    def align_center(self):
        self.move_to_world_pos([0]*3)
        
    def position_control_by_speed_integration(self, world_vel, time):
        world_pos = self.calc_pos_base2world(self._ur_controller.get_robot_current_pose()[:3])
        delta = world_vel * time
        target_world_pos = world_pos + delta
        self.move_to_world_pos(target_world_pos)
        
    def speed_control(self, world_vel, time):
        world2base_tf = self._tfBuffer.lookup_transform('world', 'base_link', rospy.Time(), timeout=rospy.Duration(4.0))
        # trans = rosnp.numpify(world2base_tf.transform.translation)
        quat = rosnp.numpify(world2base_tf.transform.rotation)
        r = R.from_quat(quat)
        base_vel = np.concatenate((r.apply(world_vel), [0.]*3))
        self._ur_controller.speedl_control(base_vel, 0.2, time)
        rospy.sleep(time+0.1)
    
    def execute(self, goal):
        if goal.type == 'o':
            self.align_center()
        elif goal.type == 's':
            world_vel = np.clip(goal.velocity[:3], -0.2, 0.2)
            step = goal.time
            # self.speed_control(world_vel, step)
            self.position_control_by_speed_integration(world_vel, step)
            rospy.sleep(step)
        elif goal.type != 'c':
            self.calibrate()
        else:
            return
        
        pose = self._ur_controller.get_robot_current_pose()
        
        self._result.completed = True
        self._result.position = self.calc_pos_base2world(pose[:3])
        self._result.orientation = pose[3:]
        rospy.loginfo('%s: succedded' % self._action_name)
        self._server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('robot_control_server')
    server = ControllerActionServer(rospy.get_name(), calib=False)
    rospy.spin()