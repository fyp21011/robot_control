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
import rospy
from std_msgs.msg import Float64

rospy.init_node("test_move", anonymous=True)
ur_control = MoveGroupPythonInteface(sim=False)  # real
rospy.loginfo('init ok.')

res = ur_control.set_speed_slider(0.6)
rospy.loginfo("set_speed_slider: {}".format(res))
rospy.sleep(1)

depth = 0.062  # -0.0255
# test pos
test1_pos = [0.413, 0.375, depth + 0.01, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05,
             5.8065731946048344e-06]
test2_pos = [0.474, 0.425, depth + 0.01, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05,
             5.8065731946048344e-06]
test3_pos = [0.53, 0.365, depth + 0.01, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05,
             5.8065731946048344e-06]
test4_pos = [0.465, 0.305, depth + 0.01, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05,
             5.8065731946048344e-06]
(plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*test1_pos))
rospy.sleep(2)
(plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*test2_pos))
rospy.sleep(2)
(plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*test3_pos))
rospy.sleep(2)
(plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*test4_pos))
rospy.sleep(2)

res = ur_control.set_speed_slider(0.25)
rospy.loginfo("set_speed_slider: {}".format(res))
rospy.sleep(1)

xyposlist = [[0.45, 0.4, 0.5, 0.34], [0.47, 0.41, 0.48, 0.33], [0.492, 0.405, 0.459, 0.329],
             [0.51, 0.38, 0.435, 0.35], [0.512, 0.36, 0.433, 0.375], [0.5, 0.34, 0.45, 0.4],
             [0.48, 0.33, 0.47, 0.41], [0.459, 0.329, 0.492, 0.405], [0.435, 0.35, 0.51, 0.38],
             [0.433, 0.375, 0.512, 0.36], [0.45, 0.4, 0.5, 0.34], [0.5, 0.34, 0.45, 0.4]]
for i in range(12):
    ready_pos = [xyposlist[i][0], xyposlist[i][1], depth + 0.05, 0.9239015802541878, -0.3826301982273228,
                 3.707584038430941e-05,
                 5.8065731946048344e-06]
    start_pos = [xyposlist[i][0], xyposlist[i][1], depth, 0.9239015802541878, -0.3826301982273228,
                 3.707584038430941e-05,
                 5.8065731946048344e-06]
    end_pos = [xyposlist[i][2], xyposlist[i][3], depth, 0.9239015802541878, -0.3826301982273228,
               3.707584038430941e-05, 5.8065731946048344e-06]
    up_pos = [xyposlist[i][2], xyposlist[i][3], depth + 0.05, 0.9239015802541878, -0.3826301982273228,
              3.707584038430941e-05,
              5.8065731946048344e-06]
    (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
    (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
    rospy.sleep(0.2)
    (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
    rospy.sleep(0.2)
    (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

    xyposlist = [[0.45, 0.4, 0.5, 0.34], [0.47, 0.41, 0.48, 0.33], [0.492, 0.405, 0.459, 0.329],
                 [0.51, 0.38, 0.435, 0.35], [0.512, 0.36, 0.433, 0.375], [0.5, 0.34, 0.45, 0.4],
                 [0.48, 0.33, 0.47, 0.41], [0.459, 0.329, 0.492, 0.405], [0.435, 0.35, 0.51, 0.38],
                 [0.433, 0.375, 0.512, 0.36], [0.45, 0.4, 0.5, 0.34], [0.5, 0.34, 0.45, 0.4]]
    for i in range(12):
        ready_pos = [xyposlist[i][0], xyposlist[i][1], depth + 0.05, 0.9239015802541878, -0.3826301982273228,
                     3.707584038430941e-05,
                     5.8065731946048344e-06]
        start_pos = [xyposlist[i][0], xyposlist[i][1], depth, 0.9239015802541878, -0.3826301982273228,
                     3.707584038430941e-05,
                     5.8065731946048344e-06]
        end_pos = [xyposlist[i][2], xyposlist[i][3], depth, 0.9239015802541878, -0.3826301982273228,
                   3.707584038430941e-05, 5.8065731946048344e-06]
        up_pos = [xyposlist[i][2], xyposlist[i][3], depth + 0.05, 0.9239015802541878, -0.3826301982273228,
                  3.707584038430941e-05,
                  5.8065731946048344e-06]
        (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
        (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
        rospy.sleep(0.2)
        (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
        rospy.sleep(0.2)
        (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', Float64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()




# #T1
# ready_pos = [0.45, 0.4, depth + 0.05 , 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.45, 0.4, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.5, 0.34, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.5, 0.34, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
#
# #T2
# ready_pos = [0.47, 0.41,depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.47, 0.41, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.48, 0.33, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.48, 0.33,depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
#
# #T3
# ready_pos = [0.492, 0.405, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.492, 0.405, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.459, 0.329, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.459, 0.329, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
#
# #T4
# ready_pos = [0.51, 0.38, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.51, 0.38, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.435, 0.35, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.435, 0.35, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
#
# #T5
# ready_pos = [0.512, 0.36, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.512, 0.36, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.433, 0.375, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.433, 0.375, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
#
# #T1'
# ready_pos = [0.45, 0.4, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.45, 0.4, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.5, 0.34, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.5, 0.34, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
#
# #T2'
# ready_pos = [0.47, 0.41, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.47, 0.41, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.48, 0.33, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.48, 0.33, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
#
# #T3'
# ready_pos = [0.492, 0.405, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.492, 0.405, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.459, 0.329, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.459, 0.329,depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
#
# #T4'
# ready_pos = [0.51, 0.38, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.51, 0.38, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.435, 0.35, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.435, 0.35, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
#
# #T5'
# ready_pos = [0.512, 0.36, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.512, 0.36, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.433, 0.375, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.433, 0.375, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
#
# #Td1
# ready_pos = [0.45, 0.4, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.45, 0.4, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.5, 0.34, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.5, 0.34, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
#
# #Td1'
# ready_pos = [0.45, 0.4, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# start_pos = [0.45, 0.4, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# end_pos = [0.5, 0.34, depth, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# up_pos = [0.5, 0.34, depth + 0.05, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*up_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*end_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*start_pos))
# (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*ready_pos))
#

