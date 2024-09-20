'''
Description: 
Version: 1.0
Autor: Julian Lin
Date: 2024-09-20 00:09:48
LastEditors: Julian Lin
LastEditTime: 2024-09-20 15:32:51
'''
import sys
import copy
import tty, termios, select
# ros
import rospy

# moveit
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

# msgs
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
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
key_mapping = {'w': (1,0,0), 's': (-1,0,0),
               'a': (0,1,0), 'd': (0,-1,0),
               'q': (0,0,1), 'e': (0,0,-1),
               }


class moveit_robot_arm_node:
    def __init__(self, node_name_, planning_time_=5, tolerance_=0.01, max_velocity_=0.1, max_acceleration_=0.1):
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node(node_name_, anonymous=True)
            self.robot = RobotCommander()
            self.scene = PlanningSceneInterface()
            self.arm_group = MoveGroupCommander("manipulator")
            self.gripper_group = MoveGroupCommander("gripper")
            self.end_effector_link = self.arm_group.get_end_effector_link()
            print("============ Reference frame: %s" % self.robot.get_planning_frame())
            # 规划设置
            self.cartesian = rospy.get_param("~cartesian", True)
            self.arm_group.allow_replanning(True)
            self.arm_group.set_planning_time(planning_time_)
            self.arm_group.set_goal_position_tolerance(tolerance_)
            self.arm_group.set_goal_orientation_tolerance(tolerance_)
            self.arm_group.set_max_velocity_scaling_factor(max_velocity_)
            self.arm_group.set_max_acceleration_scaling_factor(max_acceleration_)
            self.arm_group.set_named_target("home")
            self.arm_group.go()
            rospy.sleep(1)
            self.state = self.robot.get_current_state()
            self.goal = PoseStamped()

            # 获取规划组名称
            group_names = self.robot.get_group_names()
            print("============ Available Planning Groups:", self.robot.get_group_names())

            # 规划坐标系
            arm_planning_frame = self.arm_group.get_planning_frame()
            print("============ Planning frame: %s" % arm_planning_frame)

            eef_link = self.arm_group.get_end_effector_link()
            print("============ End effector link: %s" % eef_link)

            gripper_planning_frame = self.gripper_group.get_planning_frame()
            print("============ Gripper Planning frame: %s" % gripper_planning_frame)

            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")

            # pub
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
            plan = self.arm_group.plan()
            self.display_trajectory(plan)
            self.display_goal(self.goal)
        except Exception as e:
            rospy.logerr(f"Failed to move to goal: {str(e)}")

    def multi_points_move_to_goal(self, goal_pose_, waypoints_):
        try:
            print("============ Move to goal %s" % goal_pose_)
            print("============ Move to waypoints %s" % waypoint_ for waypoint_ in waypoints_)
            self.goal = goal_pose_
            waypoints = []
            wpose = self.arm_group.get_current_pose().pose
            for waypoint_ in waypoints_:
                wpose.position.x = waypoint_[0]
                wpose.position.y = waypoint_[1]
                wpose.position.z = waypoint_[2]
                waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            if fraction < 1.0:
                rospy.logwarn(f"Cartesian path computed with low fraction: {fraction}")
            self.arm_group.execute(plan, wait=True)
            rospy.sleep(1)
            self.display_trajectory(plan)
            self.display_goal(self.goal)
        except Exception as e:
            rospy.logerr(f"Failed to execute multi-point move: {str(e)}")

    def get_key(self, settings_):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings_)
        return key

    def keyboard_move(self, scale_=0.01):
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

    def gripper_control(self, state_):
        try:
            print("============ Gripper control %s" % state_)
            self.gripper_group.set_joint_value_target([state_])
            self.gripper_group.go()
            rospy.sleep(1)
        except Exception as e:
            rospy.logerr(f"Failed to control gripper: {str(e)}")

    def check_bounds(self, pose):
        # 简单边界检查，可以根据实际机械臂的工作空间设定具体值
        if pose.pose.position.x < -1 or pose.pose.position.x > 1:
            return False
        if pose.pose.position.y < -1 or pose.pose.position.y > 1:
            return False
        if pose.pose.position.z < 0 or pose.pose.position.z > 2:
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
