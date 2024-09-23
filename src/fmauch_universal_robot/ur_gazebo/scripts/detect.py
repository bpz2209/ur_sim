import LLM

import sys
import tty
import select
import termios

# ros
import rospy

# tf
import tf2_ros
import tf2_geometry_msgs

# msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

# opencv
import cv2
class detector:
    def __init__(self, api_key_, url_, model_name_):
        self.llm = LLM(api_key_, url_, model_name_)
        self.waypoints = []
        self.target_object_pose = PoseStamped()
        # init node
        rospy.init_node("detector", anonymous=True)

        # sub
        self.image = None
        self.depth_image = None
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_callback)

        # pub
        self.goal_publisher = rospy.Publisher('/detect_goal', PoseStamped, queue_size=20)
        
    def image_callback(self, msg):
        # ros image to opencv image
        self.image = cv2.imread(msg)

    def depth_image_callback(self, msg):
        # ros image to opencv image
        self.depth_image = cv2.imread(msg)

    def get_key(self, settings_):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings_)
        return key

    def detect(self, image_, depth_image_):
        setting = termios.tcgetattr(sys.stdin)
        while True:
            key = self.get_key(setting)
            if key == '\x03':
                break
            # 对齐image和depth_image
            try: 
                transform = self.tf_buffer.lookup_transform("d435_color_frame", "d435_depth_frame", rospy.Time(0), rospy.Duration(1.0))
                depth_image_aligned = tf2_geometry_msgs.do_transform_pose(depth_image_, transform)
            except Exception as e:
                rospy.logerr(f"Failed to align depth image to color image: {str(e)}")
                return
            response = self.llm.chat_with_image("user", "detect", image_, depth_image_)
            # TODO:提取response中的物体信息
            # TODO:提取response中的障碍物信息
            self.goal_publisher.publish(self.target_object_pose)
        
if __name__ == "__main__":
    # rospy.has_param
    print("============ param is ", rospy.has_param("/ur_gazebo"))
    debug = rospy.get_param("detect_debug")
    print("============ detect_nodo debug is ", debug)
    api_key = rospy.get_param("api_key")
    url = rospy.get_param("url")
    model_name = rospy.get_param("model_name")
    detector = detector(api_key, url, model_name)
    if debug:
        pass
    elif not debug:
        detector.detect(detector.image, detector.depth_image)


    