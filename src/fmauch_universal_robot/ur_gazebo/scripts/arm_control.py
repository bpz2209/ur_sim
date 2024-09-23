import sys
import copy
import tty, termios, select
import rospy
import tf2_ros
import tf2_geometry_msgs  # 用于进行坐标系的转换

import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
import moveit_msgs.msg

display_msg = """
    Control Your Robot Arm! author: Julian Lin
    ---------------------------
    end_effector_move: Move the end effector to the goal position
    w s: move forward/backward
    a d: move left/right
    q e: move up/down
    j k: close/open the gripper

    Moving arm:
    1 2 3 4 5 6
    z x c v b n
    space: stop
    Ctrl + C: quit
"""

key_mapping = {'w': (1, 0, 0), 's': (-1, 0, 0),
               'a': (0, 1, 0), 'd': (0, -1, 0),
               'q': (0, 0, 1), 'e': (0, 0, -1)}

class moveit_robot_arm_node:
    def __init__(self, node_name_, planning_time_=5, tolerance_=0.001, max_velocity_=0.1, max_acceleration_=0.1):
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node(node_name_, anonymous=True)
            self.robot = RobotCommander()
            self.scene = PlanningSceneInterface()
            self.arm_group = MoveGroupCommander("manipulator")
            self.gripper_group = MoveGroupCommander("gripper")
            self.end_effector_link = self.arm_group.get_end_effector_link()

            # 初始化 tf buffer 和 listener
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            
            # 规划设置
            self.arm_group.allow_replanning(True)
            self.arm_group.set_planning_time(planning_time_)
            self.arm_group.set_goal_position_tolerance(tolerance_)
            self.arm_group.set_goal_orientation_tolerance(tolerance_)
            self.arm_group.set_max_velocity_scaling_factor(max_velocity_)
            self.arm_group.set_max_acceleration_scaling_factor(max_acceleration_)

            self.state = self.robot.get_current_state()
            print("============ Current pose: %s" % self.arm_group.get_current_pose().pose)
            self.goal = PoseStamped()
            self.goal.pose = copy.deepcopy(self.arm_group.get_current_pose().pose)
            self.goal.header.stamp = rospy.Time.now()
            self.goal.header.frame_id = "base_link"  # 设置参考坐标系为 base_link

            # 获取可用规划组的名称
            group_names = self.robot.get_group_names()
            print("============ Available Planning Groups:", group_names)

            # 打印当前的规划坐标系
            arm_planning_frame = self.arm_group.get_planning_frame()
            print("============ Planning frame: %s" % arm_planning_frame)

            # 末端执行器信息
            eef_link = self.arm_group.get_end_effector_link()
            print("============ End effector link: %s" % eef_link)

            # 打印当前机器人状态
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")

            # pub
            self.display_trajectory_publisher = rospy.Publisher('/manipulator/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
            self.goal_publisher = rospy.Publisher('/manipulator/goal', PoseStamped, queue_size=20)

            # sub
            self.detect_goal_sub = rospy.Subscriber("/detect_goal", PoseStamped, self.realsense_direct_move_to_goal)

        except Exception as e:
            rospy.logerr(f"Initialization failed: {str(e)}")

    def display_trajectory(self, path):
        try:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(path)
            self.display_trajectory_publisher.publish(display_trajectory)
        except Exception as e:
            rospy.logerr(f"Failed to display trajectory: {str(e)}")

    def display_goal(self, goal):
        try:
            self.goal_publisher.publish(goal)
        except Exception as e:
            rospy.logerr(f"Failed to display goal: {str(e)}")
    
    # 从 d435_color_frame 的坐标系转换到 base_link 的坐标系
    def realsense_direct_move_to_goal(self, pose_):
        try:
            # 执行 d435_color_frame 到 base_link 的坐标系转换
            try:
                transform = self.tf_buffer.lookup_transform("base_link", "d435_color_frame", rospy.Time(0), rospy.Duration(1.0))
                transformed_goal = tf2_geometry_msgs.do_transform_pose(pose_, transform)
            except Exception as e:
                rospy.logerr(f"Failed to transform goal from d435_color_frame to base_link: {str(e)}")
            self.direct_move_to_goal(transformed_goal)
        except Exception as e:
            rospy.logerr(f"Failed to realsense move to goal: {str(e)}")

        
    def direct_move_to_goal(self, goal_pose_):
        try:
            print("============ Move to goal %s" % goal_pose_)
            self.goal = goal_pose_

            # 执行 base_link 到 world 的坐标系转换
            try:
                transform = self.tf_buffer.lookup_transform("world", "base_link", rospy.Time(0), rospy.Duration(1.0))
                transformed_goal = tf2_geometry_msgs.do_transform_pose(goal_pose_, transform)
            except Exception as e:
                rospy.logerr(f"Failed to transform goal from base_link to world: {str(e)}")
                return

            # 使用转换后的目标进行运动规划
            self.arm_group.set_pose_target(transformed_goal)

            success = self.arm_group.go(wait=True)  # 添加了成功检测
                
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            # 如果成功，显示规划路径
            self.display_trajectory(self.arm_group.get_current_joint_values())
            self.display_goal(self.goal)
            # 执行完成 return True
            if success:
                return True
            else:
                rospy.logwarn("Failed to move to the goal")
                return False
        except Exception as e:
            rospy.logerr(f"Failed to move to goal: {str(e)}")

    def get_key(self, settings_):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings_)
        return key

    def keyboard_move(self, scale_=0.1):
        settings = termios.tcgetattr(sys.stdin)
        print(display_msg)
        while not rospy.is_shutdown():
            key = self.get_key(settings)
            if key in key_mapping.keys():
                x = key_mapping[key][0] * scale_
                y = key_mapping[key][1] * scale_
                z = key_mapping[key][2] * scale_

                arm_state = self.arm_group.get_current_pose().pose
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = "base_link"  # 设置为 base_link
                goal_pose.header.stamp = rospy.Time.now()
                goal_pose.pose.position.x = arm_state.position.x + x
                goal_pose.pose.position.y = arm_state.position.y + y
                goal_pose.pose.position.z = arm_state.position.z + z
                goal_pose.pose.orientation = arm_state.orientation

                # 边界检查，防止超出工作范围
                if not self.check_bounds(goal_pose):
                    rospy.logwarn("Goal pose out of bounds!")
                    continue

                self.direct_move_to_goal(goal_pose)
            elif key == 'j':
                self.gripper_control(1)
            elif key == 'k':
                self.gripper_control(0)
            elif key == '\x03':  # 处理 Ctrl+C 退出
                break

    def gripper_control(self, state_):
        try:
            joint_names = self.gripper_group.get_active_joints()
            current_joint_values = self.gripper_group.get_current_joint_values()

            print(f"Active gripper joints: {joint_names}")
            print(f"Current gripper joint values: {current_joint_values}")

            if 'finger_joint' in joint_names:
                finger_joint_index = joint_names.index('finger_joint')

                if state_ == 1:  # 完全闭合
                    print("Closing gripper")
                    joint_goal = [0.8]
                elif state_ == 0:  # 完全打开
                    print("Opening gripper")
                    joint_goal = [0.0]
                else:
                    rospy.logwarn("Invalid state for gripper control")
                    return

                current_joint_values[finger_joint_index] = joint_goal[0]
                self.gripper_group.set_joint_value_target(current_joint_values)
                self.gripper_group.go(wait=True)
                self.gripper_group.stop()
                rospy.sleep(1)
            else:
                rospy.logwarn("'finger_joint' not found in gripper joints")
        except Exception as e:
            rospy.logerr(f"Failed to control gripper: {str(e)}")

    def check_bounds(self, pose):
        if pose.pose.position.x < -1.5 or pose.pose.position.x > 1.5:
            print("x %f" % pose.pose.position.x)
            return False
        if pose.pose.position.y < -1.5 or pose.pose.position.y > 1.5:
            print("y %f" % pose.pose.position.y)
            return False
        if pose.pose.position.z < 0 or pose.pose.position.z > 2:
            print("z %f" % pose.pose.position.z)
            return False
        return True


if __name__ == "__main__":
    try:
        node_name = "moveit_robot_arm_node"
        robot_arm = moveit_robot_arm_node(node_name)
        # rospy.has_param
        print("============ param is ", rospy.has_param("/ur_gazebo"))
        debug = rospy.get_param("controller_debug")
        print("============ arm_control_nodo debug is ", debug)
        if debug == True:
            robot_arm.keyboard_move()
        else:
            rospy.spin()
        moveit_commander.roscpp_shutdown()
        print("============ Python moveit_robot_arm_node shutdown")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")
