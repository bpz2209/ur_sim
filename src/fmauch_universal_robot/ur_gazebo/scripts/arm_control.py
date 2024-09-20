'''
Description: 
Version: 1.0
Autor: Julian Lin
Date: 2024-09-20 00:09:48
LastEditors: Julian Lin
LastEditTime: 2024-09-20 04:23:19
'''
import sys
import copy
import tty, termios, select
import rospy

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
            
            # 规划设置
            self.arm_group.allow_replanning(True)
            self.arm_group.set_planning_time(planning_time_)
            self.arm_group.set_goal_position_tolerance(tolerance_)
            self.arm_group.set_goal_orientation_tolerance(tolerance_)
            self.arm_group.set_max_velocity_scaling_factor(max_velocity_)
            self.arm_group.set_max_acceleration_scaling_factor(max_acceleration_)

            # 初始位姿设定（避免非法状态）
            init_pose = PoseStamped()
            init_pose.header.frame_id = self.robot.get_planning_frame()
            init_pose.header.stamp = rospy.Time.now()
            init_pose.pose.position.x = 0.4  # 确保在有效工作范围内
            init_pose.pose.position.y = 0.0
            init_pose.pose.position.z = 0.4
            init_pose.pose.orientation.w = 1.0  # 朝向设为单位四元数
            
            # self.arm_group.set_pose_target(init_pose)
            # self.arm_group.go(wait=True)
            # rospy.sleep(1)

            self.state = self.robot.get_current_state()
            # get the current pose of the end effector
            print("============ Current pose: %s" % self.arm_group.get_current_pose().pose)
            self.goal = PoseStamped()
            self.goal.pose = copy.deepcopy(self.arm_group.get_current_pose().pose)
            self.goal.header.stamp = rospy.Time.now()
            self.goal.header.frame_id = self.robot.get_planning_frame()

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

            # 发布器
            self.display_trajectory_publisher = rospy.Publisher('/manipulator/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
            self.goal_publisher = rospy.Publisher('/manipulator/goal', PoseStamped, queue_size=20)

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

    def direct_move_to_goal(self, goal_pose_):
        try:
            print("============ Move to goal %s" % goal_pose_)
            self.goal = goal_pose_
            self.arm_group.set_pose_target(self.goal)

            success = self.arm_group.go(wait=True)  # 添加了成功检测
            if not success:
                rospy.logwarn("Failed to move to the goal")
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            rospy.sleep(1)
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
                goal_pose.header.frame_id = self.robot.get_planning_frame()
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
            # 获取当前抓取器的关节名称和当前关节值
            joint_names = self.gripper_group.get_active_joints()
            current_joint_values = self.gripper_group.get_current_joint_values()

            print(f"Active gripper joints: {joint_names}")
            print(f"Current gripper joint values: {current_joint_values}")

            # 确保 'finger_joint' 存在于抓取器的关节列表中
            if 'finger_joint' in joint_names:
                # 获取 'finger_joint' 的索引
                finger_joint_index = joint_names.index('finger_joint')

                # 设置抓取器的目标值：0.0 表示完全打开，0.8 表示完全闭合（根据实际抓取器调整）
                if state_ == 1:  # 完全闭合
                    print("Closing gripper")
                    joint_goal = [0.8]  # 完全闭合的目标值
                elif state_ == 0:  # 完全打开
                    print("Opening gripper")
                    joint_goal = [0.0]  # 完全打开的目标值
                else:
                    rospy.logwarn("Invalid state for gripper control")
                    return

                # 设置 'finger_joint' 的目标值
                current_joint_values[finger_joint_index] = joint_goal[0]

                # 应用目标值
                self.gripper_group.set_joint_value_target(current_joint_values)
                self.gripper_group.go(wait=True)
                self.gripper_group.stop()
                rospy.sleep(1)
            else:
                rospy.logwarn("'finger_joint' not found in gripper joints")
        except Exception as e:
            rospy.logerr(f"Failed to control gripper: {str(e)}")

    def check_bounds(self, pose):
        # 根据实际机械臂的工作空间设定边界值
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
        robot_arm.keyboard_move()
        moveit_commander.roscpp_shutdown()
        print("============ Python moveit_robot_arm_node shutdown")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")
