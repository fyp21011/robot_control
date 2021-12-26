import copy
import threading

# import apriltag_ros.msg
import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseArray, TransformStamped
from std_msgs.msg import String

import helper
from tf import transformations as tfs
from scene_helper import setup_scene
from ur_move import MoveGroupPythonInteface

class listener():
    def __init__(self):
        # self.tfb = tf.TransformBroadcaster()
        # self.stfb = tf2_ros.StaticTransformBroadcaster()
        # listener = tf.TransformListener()
        #
        # # Get static tf
        # listener.waitForTransform('base', 'base_link', rospy.Time(0), rospy.Duration(1))
        # tr_base2baselink = listener.lookupTransform('base', 'base_link', rospy.Time(0))
        # tf_base2baselink = listener.fromTranslationRotation(*tr_base2baselink)
        # self.tf_base2baselink = tf_base2baselink
        #
        # listener.waitForTransform('base_link', 'camera', rospy.Time(0), rospy.Duration(1))
        # tr_baselink2camera = listener.lookupTransform('base_link', 'camera', rospy.Time(0))
        # tf_baselink2camera = listener.fromTranslationRotation(*tr_baselink2camera)
        # self.tf_baselink2cam = tf_baselink2camera
        #
        # self.tf_ee2tool = helper.xyzrpy2mat44([0.09, 0, 0, np.deg2rad([-90, 0, -90])])
        # self.tf_tool2ee = np.linalg.inv(self.tf_ee2tool)
        #
        # xyzquat = helper.mat44_to_xyzquat(self.tf_ee2tool)
        # static_transformStamped = TransformStamped()
        # static_transformStamped.header.stamp = rospy.Time.now()
        # static_transformStamped.header.frame_id = 'ee_link'
        # static_transformStamped.child_frame_id = 'tool_vacuum'
        # static_transformStamped.transform.translation.x=xyzquat[0]
        # static_transformStamped.transform.translation.y=xyzquat[1]
        # static_transformStamped.transform.translation.z=xyzquat[2]
        # static_transformStamped.transform.rotation.x=xyzquat[3]
        # static_transformStamped.transform.rotation.y=xyzquat[4]
        # static_transformStamped.transform.rotation.z=xyzquat[5]
        # static_transformStamped.transform.rotation.w=xyzquat[6]
        # self.stfb.sendTransform(static_transformStamped)
        #
        # self.tf_listener = listener

        # self.count = 0

        # self.plot_pub = rospy.Publisher("plot", geometry_msgs.msg.PointStamped, queue_size=2)

        # ns = '/obj_detect/'
        self.obj_pose_sub = rospy.Subscriber('/tag_detections', apriltag_ros.msg.AprilTagDetectionArray, self.detect_callbak, queue_size=1)

        # self.lock_read = threading.Lock()
        # self.run_flag = False
        # self.gripper_pos = 80   # !!!!!!!!!!! Default grip pos
        # self.chat_sub = rospy.Subscriber("/debug_chat", String, self.chat_callbak, queue_size=1)
        # self.posearray_sub = rospy.Subscriber("/objpose", PoseArray, self.pose_callbak, queue_size=2)
        # self.pose_array=None

    def detect_callbak(self,msg):
        # msg = apriltag_ros.msg.AprilTagDetectionArray()
        # for det in msg.detections:
        #     det.pose
        a=msg.detections[0].pose.pose.pose
        rospy.loginfo(msg)

    def chat_callbak(self, msg):
        str_msg = msg.data
        if str_msg == '1':
            self.lock_read.acquire()
            self.run_flag = True
            self.lock_read.release()
            rospy.loginfo("Recieve run flag")

    def read_and_reset_run(self):
        with self.lock_read:
            run_flag = self.run_flag
            self.run_flag = False
        return run_flag

    def pose_callbak(self, msg):
        rospy.loginfo_throttle(10, "recieve pose")

        pose_array=[]
        for pose in msg.poses:
            tf_cam2obj = helper.pose_to_mat44(pose)
            # tf_obj2ee = helper.xyzrpy2mat44([0,0,0,*np.deg2rad([90, -90, 0])])
            tf_baselink2obj = self.tf_baselink2cam.dot(tf_cam2obj).dot(self.tf_tool2ee)
            pose_l = helper.mat44_to_xyzquat(tf_baselink2obj)
            pose_array.append(pose_l)

        xyzquat = pose_array[0]
        self.tfb.sendTransform(xyzquat[:3],
                          xyzquat[3:],
                          rospy.Time.now(),
                          'target',
                          'base_link')

        with self.lock_read:
            self.pose_array = pose_array

    def read_pose_array(self):
        with self.lock_read:
            return copy.deepcopy(self.pose_array)


if __name__ == '__main__':
    rospy.init_node("test_move")

    # listener = listener()
    # rospy.sleep(1)
    # rospy.spin()

    # ur_control = MoveGroupPythonInteface(sim=True)  #simu
    ur_control = MoveGroupPythonInteface(sim=False)  #real
    rospy.loginfo('init ok.')

    ur_control.group.get_planning_frame()
    ur_control.group.get_end_effector_link()

    ur_control.remove_allobjects()
    # setup_scene(ur_control)

    # Config UR
    res = ur_control.play_program()
    rospy.loginfo("play_program: {}".format(res))

    res = ur_control.set_speed_slider(0.8)
    rospy.loginfo("set_speed_slider: {}".format(res))
    rospy.sleep(1)

    # obtain the current pose list
    rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    rospy.loginfo('current joint: \n{}'.format(ur_control.get_joint_pos_list()))

    xyzquat = ur_control.get_pose_list()
    T_baselink2tool = helper.xyzquat_to_mat44(xyzquat)
    xyzrpy_deg = helper.mat44_to_xyzrpy_deg(T_baselink2tool)
    xyzrpy = copy.deepcopy(xyzrpy_deg)
    xyzrpy[3:] = np.deg2rad([-180,0,-45])
    pose_goal = helper.mat44_to_xyzquat(helper.xyzrpy2mat44(xyzrpy))
    ready_pos = [0.45, 0.4, 0.13, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
    end_pos = [0.55, 0.3, 0.08, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
    start_pos = [0.45, 0.4, 0.08, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
    waypoints = []
    waypoints.append(helper.gen_pose(*ready_pos))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    waypoints = []
    waypoints.append(helper.gen_pose(*start_pos))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    waypoints = []
    waypoints.append(helper.gen_pose(*end_pos))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    waypoints = []
    waypoints.append(helper.gen_pose(*ready_pos))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints)


    JS_START =[0.4438713490962982, -1.3743627707110804, 1.543337345123291, 1.33539879322052, 1.5996168851852417, 2.806729793548584]
    ur_control.go_to_joint_state(*JS_START)
    # rotation matrix from box {b} to the robot {r}
    elem = np.sqrt(2)/2
    rRb = np.array([[elem, elem],
                    [-elem, elem]])
    lx = 0.28
    ly = 0
    forward_vec_b = np.array([lx, ly])
    forward_vec_r = (np.dot(rRb, forward_vec_b)).tolist()

    waypoints = []
    # start pt1
    start1_pose = [0.5285595797087597, 0.33286161521460467, 0.1501572777718378, -0.9219784480530051, 0.38460944184461326, 0.03746876472868755, 0.02504815840443044]
    #start1_pose[2] -= 0.017
    waypoints.append(helper.gen_pose(*start1_pose))

    ur_control.go_to_pose_goal(helper.gen_pose(*start1_pose))

    (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    if plan is None:
        rospy.logerr('plan traj failed')

    # end pt1
    end1_pose = copy.deepcopy(start1_pose)
    end1_pose[0] += forward_vec_r[0]
    end1_pose[1] += forward_vec_r[1]
    waypoints.append(helper.gen_pose(*end1_pose))

    # # start pt2
    # lx = 0.0
    # ly = 0.06
    # slide_vec_b = np.array([lx, ly])
    # slide_vec_r = (np.dot(rRb, slide_vec_b)).tolist()
    # start2_pose = copy.deepcopy(start1_pose)
    # start2_pose[0] += slide_vec_r[0]
    # start2_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*start2_pose))
    #
    # # end pt2
    # end2_pose = copy.deepcopy(end1_pose)
    # end2_pose[0] += slide_vec_r[0]
    # end2_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*end2_pose))

    # # start pt3
    # start3_pose = copy.deepcopy(start2_pose)
    # start3_pose[0] += slide_vec_r[0]
    # start3_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*start3_pose))
    # # end pt3
    # end3_pose = copy.deepcopy(end2_pose)
    # end3_pose[0] += slide_vec_r[0]
    # end3_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*end3_pose))

    rospy.loginfo("Start Loop ============================")
    rate = rospy.Rate(30.0)
    depth = 0
    cnt = 0

    # while not rospy.is_shutdown():
    for i in range(100):
        wpose = []
        depth = -0.005 # 5mm deeper
        for waypt in waypoints:
            waypt.position.z += depth
            wpose.append(copy.deepcopy(waypt))

        (plan, fraction) = ur_control.go_cartesian_path(wpose)
        rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
        if plan is None:
            rospy.logerr('plan traj failed')
            exit(0)


    #
    # start1_pose = [0.2700405492584124, 0.37238332698040844, 0.07725913594747941, -0.2606191842360539, 0.6443504900377716, 0.25797526467862947, 0.671072909310314]
    # end1_pose = [0.44333889398164206, 0.18649866354520614, 0.0615933580899586, -0.27145920224433784, 0.6725163143781285, 0.24662990765052872, 0.6428105452342999]
    # start2_pose = [0.27000404407539536, 0.3723622577321929, 0.05721394410574203, -0.27135752101103977, 0.6724648950007767, 0.24655319614995433, 0.6429366860357089]
    # end2_pose = [0.5261471413079262, 0.26371751069459637, 0.05571437862960854, -0.27119477840445955, 0.6724445095241685, 0.24677723247904926, 0.6429407214564483]
    #
    # goal_pose = copy.deepcopy(start1_pose)
    # waypoints = helper.gen_pose(*goal_pose)
    # waypoints.position.z += 0.02  # First move up (z)
    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    # if plan is None:
    #     rospy.logerr('plan traj failed')
    # exit(0)



    # # JS_START = [-1.6007002035724085, -1.7271001974688929, -2.2029998938189905, -0.8079999128924769, 1.5951000452041626, -0.03099996248354131]
    # # JS_START = [0.5693637728691101, -1.0146344343768519, 1.6008195877075195, -0.5721419493304651, 1.4757764339447021, 1.5631433725357056]
    # JS_START = [0.7037186026573181, -1.1263330618487757, 1.8121705055236816, -0.614563290272848, 1.4962608814239502, 1.542791485786438]
    #
    # rospy.loginfo("go to init joint")
    # ur_control.go_to_joint_state(*JS_START)
    # rospy.sleep(1)
    #
    # #
    # res = ur_control.set_speed_slider(0.65)  #speed control
    # rospy.loginfo("set_speed_slider: {}".format(res))
    # rospy.sleep(1)
    #
    # rospy.loginfo("Start Loop ============================")
    # rate = rospy.Rate(30.0)
    # while not rospy.is_shutdown():
    #     run = listener.read_and_reset_run()
    #     if True or run:
    #         # pass
    #         # rospy.loginfo("Run --------------------")
    #         poses = listener.read_pose_array()
    #         if poses is None:
    #             rate.sleep()
    #             continue
    #         rospy.loginfo("Rec")
    #         (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*(poses[0])))
    #         if plan is None:
    #             rospy.logerr('plan to init pose failed')
    #
    #         waypoints = []
    #         for pose in poses:
    #             # pose = helper.xyzquat_to_mat44(pose)
    #             # pose = pose.dot(tfs.euler_matrix(*np.deg2rad([180, 0, 0])))
    #             # pose = helper.mat44_to_xyzquat(pose)
    #             waypoints.append(helper.gen_pose(*pose))
    #
    #         goal_pose =[0.35339544678026097, 0.4526103713972721, 0.12411307106246072, -0.6702878730504156, -0.247036939956884, 0.28816145022362866, 0.6376910663819154]
    #         waypoints = helper.gen_pose(*goal_pose)
    #         (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    #         if plan is None:
    #             rospy.logerr('plan traj failed')
    #         exit(0)
    #     rate.sleep()

    # ur_control.group.get_named_target_values('up')
    # ur_control.group.get_remembered_joint_values()
    # ur_control.group.set_joint_value_target('home')

    # ur_control.group.set_pose_reference_frame('base_link')

    # initial_pose = helper.gen_pose(*poses[0])
    # (plan, fraction) = ur_control.go_cartesian_path(initial_pose)


    rospy.loginfo('shut down')
