import sys
import rospy
import numpy as np
import math
import serial
import moveit_commander
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R
from dianyunceshi.msg import tomato_poseGoal 
numm=0
xxxx=0
class IntegratedActionServer(object):
    def __init__(self):
        rospy.loginfo("Initializing Integrated Action Server...")
        self.T_cam_to_ee = self.quaternion_to_rotation_matrix()
        self.listener = tf.TransformListener()
        self.initialize_moveit()
        rospy.Subscriber("best_tomato_pose", tomato_poseGoal, self.execute_cb,queue_size=1)
        rospy.spin()
        # self.execute_cb()

    def initialize_moveit(self):
        """
        初始化MoveIt! 并设置相关参数。
        """
        rospy.loginfo("Initializing MoveIt!...")
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("jaka_zu3")
        self.move_group.set_planner_id("RRT")
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.02)
        self.move_group.set_planning_time(2)
        self.move_group.set_max_acceleration_scaling_factor(0.3)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.allow_replanning(True)
        rospy.loginfo("MoveIt! initialized successfully.")
    def get_current_pose_tf(self, base_frame="base_link", ee_frame="pick"):
        """
        使用 TF 获取当前末端姿态，返回 geometry_msgs/Pose。
        
        :param base_frame: 机器人基座坐标系 (通常是 "base_link" 或 "world")
        :param ee_frame:   末端执行器坐标系 (如 "tool0", "ee_link" 等)
        :return:           geometry_msgs.Pose 或 None
        """
        try:
            # 等待 transform 准备就绪
            self.listener.waitForTransform(
                base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            (trans, rot) = self.listener.lookupTransform(
                base_frame, ee_frame, rospy.Time(0)
            )

            # 将 TF 结果转换为 geometry_msgs/Pose
            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get current pose from TF: {e}")
            return None
    def compute_interpolated_waypoints(self, start_pose, target_pose, step=0.005):
        waypoints = []
        waypoints.append(start_pose)
        dx = target_pose.position.x - start_pose.position.x
        dy = target_pose.position.y - start_pose.position.y
        dz = target_pose.position.z - start_pose.position.z
        distance = math.sqrt(dy**2)

        if distance < 1e-6:
            return waypoints

        steps = int(math.floor(distance / step)) # 计算步数
        if steps>1:
            for i in range(1, 1 + 1):
                fraction = float(i) / steps
                new_pose = Pose()
                new_pose.position.x = target_pose.position.x
                new_pose.position.y = start_pose.position.y + fraction * dy
                new_pose.position.z =target_pose.position.z 
                new_pose.orientation = start_pose.orientation  # 姿态保持不变
                waypoints.append(new_pose)
        else:
            new_pose = Pose()
            new_pose.position.x = target_pose.position.x
            new_pose.position.y = target_pose.position.y
            new_pose.position.z =target_pose.position.z 
            new_pose.orientation = start_pose.orientation  # 姿态保持不变
            waypoints.append(new_pose)           


        return waypoints

    def execute_cb(self,goal_msg):
        global numm
        global xxxx
        rospy.loginfo("Executing callback function...")

        start_time = rospy.Time.now().to_sec()
        
        # 这里假设我们已经计算出 robot_points，是需要到达的目标位置
        robot_points = self.transform_point([goal_msg.x,goal_msg.y,goal_msg.z])
        quaternion = Quaternion(0.70559, -0.0617, -0.70416, -0.049894)
        
        
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])   
        movements = np.array([0, 0.14, 0])
        # rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        displacement=rotation.apply(movements)
        # rospy.sleep(2)
        current_pose = self.move_group.get_current_pose().pose
        if numm==0:
            pose_target = Pose(Point(robot_points[0] + displacement[0],
                                    robot_points[1] + displacement[1],
                                    robot_points[2] + displacement[2]), quaternion)   
            self.move_group.set_pose_target(pose_target)
            plan = self.move_group.plan()  
            if plan[0]: 
                self.move_group.execute(plan[1], wait=True)
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                # rospy.sleep(0.5)
                numm=1    
                xxxx=  robot_points[2] 
        
        
        if robot_points is None:
            rospy.logerr("Failed to transform points. Exiting execution.")
            return

        # 假设我们已经确认需要的四元数
        quaternion = Quaternion(0.70559, -0.0617, -0.70416, -0.049894)
        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = robot_points
        target_pose.orientation = quaternion
        # target_pose.position.y=xxxx

        # 记录一下耗时（观察对比）
        rospy.loginfo(f"Time after target_pose assignment: {rospy.Time.now().to_sec() - start_time:.4f} s")
        # rospy.sleep(0.01)
        current_pose = self.get_current_pose_tf()
        # print(current_pose)
        rospy.loginfo(f"Time after get_current_pose_tf(): {rospy.Time.now().to_sec() - start_time:.4f} s")
        
        if not current_pose:
            rospy.logerr("Unable to retrieve current pose from TF. Aborting.")
            return
        # 计算从当前姿态到目标姿态的插值路径
        # waypoints=[current_pose,target_pose]
        waypoints = self.compute_interpolated_waypoints(current_pose, target_pose, step=0.02)
        rospy.loginfo(f"Time after compute_interpolated_waypoints(): {rospy.Time.now().to_sec() - start_time:.4f} s")
        if not waypoints:
            rospy.logwarn("No waypoints generated, skipping execution.")
            return
        # print(waypoints)

        start_time = rospy.Time.now().to_sec()
        attempt_count = 0
        max_attempts = 20

        start_time = rospy.Time.now().to_sec()
        while attempt_count < max_attempts:
            attempt_count += 1

            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.02, False)  
            if fraction < 1.0:
                continue
            self.move_group.execute(plan, wait=False)
            rospy.sleep(0.4)
            rospy.loginfo(f"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx: {rospy.Time.now().to_sec() - start_time:.4f} s")
            break
        rospy.loginfo("Execution completed successfully.")
        return
        
        
        # self.move_group.set_pose_target(target_pose)
        # plan = self.move_group.plan()  
        # if plan[0]: 
        #     self.move_group.execute(plan[1], wait=True)
        #     self.move_group.stop()
        #     self.move_group.clear_pose_targets()
        #     rospy.sleep(4)        
        
        
        
        
        
        
        
        
    def quaternion_to_rotation_matrix(self):
        """
        计算从四元数到旋转矩阵的转换。
        """
        q = [0.464673,0.5162789,-0.5086267,0.50875689]
        rotation_matrix = quaternion_matrix(q)[:3, :3]
        T_cam_to_ee = np.eye(4)
        T_cam_to_ee[:3, :3] = rotation_matrix
        T_cam_to_ee[:3, 3] = [-0.105,0.0917,0.0817]
        return T_cam_to_ee

    def transform_point(self, point_camera, target_frame='base_link', source_frame_ee='pick'):
        """
        将相机坐标系中的点转换到机器人基座坐标系。
        """
        try:
            self.listener.waitForTransform(target_frame, source_frame_ee, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform(target_frame, source_frame_ee, rospy.Time(0))
            T_ee_to_base = tf.transformations.quaternion_matrix(rot)
            T_ee_to_base[:3, 3] = trans
            P_camera = np.array([*point_camera, 1.0])
            P_ee = self.T_cam_to_ee @ P_camera
            P_base = T_ee_to_base @ P_ee
            return P_base[:3]
        except tf.Exception as e:
            rospy.logerr(f"Transformation error: {e}")
            return None

if __name__ == '__main__':
    rospy.init_node('integrated_action_server')
    server = IntegratedActionServer()
    