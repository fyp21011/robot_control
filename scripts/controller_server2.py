#! /usr/bin/env python
from __future__ import print_function
from pdb import set_trace as breakpoint
import csv
import numpy as np
# from numpy import linalg
from scipy.spatial.transform import Rotation as R
import ros_numpy as rosnp
import rospy
import tf2_ros as tf2
from tf import transformations as ts
import geometry_msgs.msg
from tf2_geometry_msgs import PoseStamped
import actionlib
from robot_control.msg import ControlAction, ControlFeedback, ControlResult
from ur_move import MoveGroupPythonInteface
import helper


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
    _calib_filename = 'calib_data2.npz'
    _feedback = ControlFeedback()
    _result = ControlResult()
    
    def __init__(self, name, calib=True, sim=True):
        self._action_name = name
        self._ur_controller = MoveGroupPythonInteface(sim=sim)
        print(self._ur_controller.get_robot_current_pose())
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
        world2base_trans = data['trans']
        world2base_rot = data['rot']
        self.send_world2base_tf(world2base_trans, world2base_rot)
            
    def calibrate(self):
        data = np.load(self._calib_filename)
        o_pos = data['o_pos']
        
        raw_input('move robot to the center of origin in world frame')
        pose = self._ur_controller.get_robot_current_pose()
        o_pos = pose[:3]
        
        base_orientation = pose[3:]
        
        base_orientation_downward = [1., 0., 0., 0.,]
        self.move_to_base_pose(np.concatenate((o_pos, base_orientation_downward)))
        raw_input('rotate wrist 3 to desired angle')
        
        world2base_rot = base_orientation
        print('world2base_rot', world2base_rot)
        world2base_rot = ts.quaternion_multiply(base_orientation, base_orientation_downward)
        
        # translation
        world2base_trans = o_pos
        # world2base_trans[2] += 0.005 # safety margin
        
        np.savez(self._calib_filename, 
                 orientation=base_orientation, 
                 o_pos=o_pos, 
                 trans=world2base_trans, 
                 rot=world2base_rot, 
                 )
        
        self.send_world2base_tf(world2base_trans, world2base_rot)
    
    def send_world2base_tf(self, trans, rot):
        b = tf2.StaticTransformBroadcaster()
        m = geometry_msgs.msg.TransformStamped()
        m.header.frame_id = 'world'
        m.child_frame_id = 'base_link'
        m.transform.translation = rosnp.msgify(geometry_msgs.msg.Vector3, trans)
        m.transform.rotation = rosnp.msgify(geometry_msgs.msg.Quaternion, rot)
        b.sendTransform(m)
    
    def transform_pose(self, target_frame, source_frame, pose):
        #TODO: check why target_frame and source_frame should be flipped?
        assert len(pose) == 7
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        # ps.header.frame_id = source_frame
        ps.header.frame_id = target_frame
        ps.pose.position = rosnp.msgify(geometry_msgs.msg.Vector3, pose[:3])
        ps.pose.orientation = rosnp.msgify(geometry_msgs.msg.Quaternion, pose[3:])
        
        # res = self._tfBuffer.transform(ps, target_frame, timeout=rospy.Duration(4.0))
        res = self._tfBuffer.transform(ps, source_frame, timeout=rospy.Duration(4.0))
        
        #HACK: flip the orientation again for TCP to remain facing downwards
        res.pose.orientation = rosnp.msgify(geometry_msgs.msg.Quaternion, 
                                            ts.quaternion_multiply(rosnp.numpify(res.pose.orientation), [-1.,0.,0.,0.]))
        
        return np.concatenate((rosnp.numpify(res.pose.position), rosnp.numpify(res.pose.orientation)))
        
    def move_to_base_pose(self, base_pose):
        plan, fraction = self._ur_controller.plan_cartesian_path(helper.gen_pose(*base_pose))
        self._ur_controller.display_trajectory(plan)
        # raw_input('click enter to move robot')
        self._ur_controller.go_cartesian_path(helper.gen_pose(*base_pose))
    
    def move_to_world_pose(self, world_pose):
        """Clip the target position within the world boundary, and move robot to the position

        Args:
            world_pos (array_like): 3*1 array of target world position
        """
        world_pose[:2] = np.clip(world_pose[:2], -0.2, 0.2)
        world_pose[3] = np.clip(world_pose[2], 0., 0.4)
        base_pose = self.transform_pose('base_link', 'world', world_pose)
        self.move_to_base_pose(base_pose)
    
    def align_center(self):
        self.move_to_world_pose(np.array([0.,0.,0.,0.,0.,0.,1.]))

        
    def position_control_by_speed_integration(self, world_vel, time):
        base_pose = self._ur_controller.get_robot_current_pose()
        world_pose = self.transform_pose('world', 'base_link', base_pose)
        
        world_pos = world_pose[:3]
        delta_pos = world_vel[:3] * time
        target_world_pos = world_pos + delta_pos
        
        world_orientation = world_pose[3:]
        delta_rot = world_vel[3:] * time
        delta_rot_q = ts.quaternion_from_euler(*delta_rot)
        target_world_orientation = ts.quaternion_multiply(world_orientation, delta_rot_q)
        
        target_world_pose = np.concatenate((target_world_pos, target_world_orientation))
        
        self.move_to_world_pose(target_world_pose)
        
    def speed_control(self, world_vel, time):
        world2base_tf = self._tfBuffer.lookup_transform('world', 'base_link', rospy.Time(), timeout=rospy.Duration(4.0))
        quat = rosnp.numpify(world2base_tf.transform.rotation)
        r = R.from_quat(quat)
        base_vel = np.concatenate((r.apply(world_vel), [0.]*3))
        self._ur_controller.speedl_control(base_vel, 0.2, time)
        rospy.sleep(time+0.1)
    
    def append_joint_to_csv(self):
        joint_values = self._ur_controller.group.get_current_joint_values()
        with open('joint_data.csv', 'a+ ') as f:
            writer = csv.writer(f, delimiter=' ')
            writer.writerow(joint_values) 
    
    def execute(self, goal):
        if goal.type == 'o':
            self.align_center()
        elif goal.type == 's':
            assert goal.velocity[2] == 0. # allow x,y translation only
            assert goal.velocity[3] == 0. and goal.velocity[4] == 0. # allow yaw rotation only
            world_vel = np.concatenate((np.clip(goal.velocity[:3], -0.2, 0.2), goal.velocity[3:])) # np.clip(goal.velocity[3:], -1., 1.)))
            step = goal.time
            # self.speed_control(world_vel, step)
            self.position_control_by_speed_integration(world_vel, step)
            rospy.sleep(step)
        elif goal.type == 'pw':
            self.move_to_world_pose(np.array(goal.pose))
        elif goal.type == 'pb':
            self.move_to_base_pose(np.array(goal.pose))
        elif goal.type != 'c':
            return
        
        base_pose = self._ur_controller.get_robot_current_pose()
        world_pose = self.transform_pose('world', 'base_link', base_pose)
        
        self._result.completed = True
        self._result.position = world_pose[:3]
        self._result.orientation = world_pose[3:]
        self._result.base_pose = base_pose
        rospy.loginfo('%s: succedded' % self._action_name)
        self._server.set_succeeded(self._result)
        # self.append_joint_to_csv()

if __name__ == '__main__':
    rospy.init_node('robot_control_server')
    server = ControllerActionServer(rospy.get_name(), calib=False, sim=False)
    rospy.spin()