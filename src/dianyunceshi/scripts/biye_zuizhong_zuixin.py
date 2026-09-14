import cv2
import torch
import numpy as np
from ultralytics.data.augment import LetterBox
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.cfg.models.utils.datasets import letterbox
from ultralytics.cfg.models.utils.general import check_img_size, non_max_suppression_face, scale_coords, xyxy2xywh
from ultralytics.cfg.models.utils.torch_utils import select_device, load_classifier, time_synchronized
import math
import copy
import time
import random as random1
import open3d as o3d
import rospy
import actionlib
import os
from sensor_msgs.msg import Image
import cv_bridge
from sklearn.cluster import DBSCAN
import message_filters
from ultralytics.utils import LOGGER, ops
# 导入自定义消息（请根据实际消息字段调整）
from dianyunceshi.msg import tomato_poseAction, tomato_poseGoal, posev, posevs

global_cluster_colors = {}

# --------------------- 以下为你原有的各函数定义 ---------------------
def create_direction_arrow(origin, direction, length=0.1, color=[1, 0, 0]):
    """
    根据采摘方向生成一个箭头：
      - direction==0: 采摘方向为正前方 (+Z)
      - direction==1: 从正前方向左偏45° 
      - direction==2: 从正前方向右偏45°
    """
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.005,
        cone_radius=0.01,
        cylinder_height=length * 0.8,
        cone_height=length * 0.2
    )
    if direction == 0:
        angle = 0.0
    elif direction == 1:
        angle = -np.pi/4
    elif direction == 2:
        angle = np.pi/4
    else:
        angle = 0.0

    R = np.array([[np.cos(angle), 0, np.sin(angle)],
                  [0,             1,             0],
                  [-np.sin(angle),0, np.cos(angle)]])
    arrow.rotate(R, center=(0, 0, 0))
    direction_vector = R.dot(np.array([0, 0, 1]))
    T = np.array(origin) - R.dot(np.array([0, 0, length])) - 0.1 * direction_vector
    arrow.translate(T)
    arrow.paint_uniform_color(color)
    return arrow

def collect_roi_pointcloud(orgimg, depth_img, bbox, camera_intrinsics, max_depth=1, color=[1,0,0]):
    x1, y1, x2, y2 = bbox
    H, W = depth_img.shape[:2]
    x1, x2 = max(0, x1), min(x2, W)
    y1, y2 = max(0, y1), min(y2, H)
    pts, cols = [], []
    fx, fy = camera_intrinsics[0,0], camera_intrinsics[1,1]
    cx, cy = camera_intrinsics[0,2], camera_intrinsics[1,2]
    for v in range(y1, y2):
        for u in range(x1, x2):
            d_val = depth_img[v, u]
            if d_val == 0:
                continue
            Z = d_val / 1000.0
            if Z > max_depth:
                continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            if not (np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z)):
                continue
            pts.append([X, Y, Z])
            cols.append(color)
    if len(pts) == 0:
        print(f"ROI {bbox} => no valid points!")
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(pts, dtype=float))
    pcd.colors = o3d.utility.Vector3dVector(np.array(cols, dtype=float))
    return pcd

def create_sphere_marker(center, radius=0.01, color=[0,1,0]):
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh.translate(center)
    mesh.paint_uniform_color(color)
    return mesh

def fit_sphere_least_squares(points):
    X = points[:,0]
    Y = points[:,1]
    Z = points[:,2]
    A = np.column_stack((-2*X, -2*Y, -2*Z, np.ones(len(points))))
    E = X**2 + Y**2 + Z**2
    try:
        sol, residuals, rank, s = np.linalg.lstsq(A, E, rcond=None)
        a, b, c, d = sol
        cx = -a/2
        cy = -b/2
        cz = -c/2
        r = math.sqrt(cx**2 + cy**2 + cz**2 - d)
        return cx, cy, cz, r
    except:
        return None

def fit_sphere_ransac(points, dist_threshold=0.01, max_iterations=1000, min_inliers=20):
    best_inliers = []
    best_model = None
    N = len(points)
    if N < 4:
        return None
    for it in range(max_iterations):
        sample_idx = random1.sample(range(N), 4)
        sample_points = points[sample_idx, :]
        model = fit_sphere_least_squares(sample_points)
        if model is None:
            continue
        cx, cy, cz, r = model
        dist = np.abs(np.linalg.norm(points - np.array([cx, cy, cz]), axis=1) - r)
        inliers_mask = dist < dist_threshold
        inliers_idx = np.where(inliers_mask)[0]
        inliers_count = np.sum(inliers_mask)
        if inliers_count > len(best_inliers):
            best_inliers = inliers_idx
            best_model = (cx, cy, cz, r)
            if inliers_count >= min_inliers:
                break
    if best_model is None:
        return None
    final_inliers = points[best_inliers, :]
    final_model = fit_sphere_least_squares(final_inliers)
    return final_model

def collect_tomato_points(depth_img, bbox, max_depth=1, camera_intrinsics=None):
    x1, y1, x2, y2 = bbox
    h, w = depth_img.shape[:2]
    x1, x2 = max(0, x1), min(w, x2)
    y1, y2 = max(0, y1), min(h, y2)
    points_3d = []
    fx = camera_intrinsics[0,0]
    fy = camera_intrinsics[1,1]
    cx = camera_intrinsics[0,2]
    cy = camera_intrinsics[1,2]
    for v in range(y1, y2):
        for u in range(x1, x2):
            d_val = depth_img[v, u]
            if d_val == 0:
                continue
            Z = d_val / 1000.0
            if Z > max_depth:
                continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points_3d.append((X, Y, Z))
    return np.array(points_3d, dtype=np.float32)

def show_in_camera_view(geom_list, W, H, cam_intrinsic, extrinsic=None):
    import open3d as o3d
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1280, height=960)
    for g in geom_list:
        vis.add_geometry(g)
    ctr = vis.get_view_control()
    if cam_intrinsic.shape == (3,3):
        fx = cam_intrinsic[0,0]
        fy = cam_intrinsic[1,1]
        cx = cam_intrinsic[0,2]
        cy = cam_intrinsic[1,2]
    else:
        fx, fy, cx, cy = cam_intrinsic
    param = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
    if extrinsic is None:
        extrinsic = np.eye(4, dtype=float)
    # 直接用 Open3D 的函数设置
    vis.get_view_control().convert_from_pinhole_camera_parameters(
        o3d.camera.PinholeCameraParameters(intrinsic=param, extrinsic=extrinsic)
    )
    vis.run()
    vis.destroy_window()

def points6d_to_open3d(points6d):
    pcd = o3d.geometry.PointCloud()
    points = points6d[:, :3]
    colors = points6d[:, 3:6]
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    return pcd

def collect_colored_roi_points(color_img, depth_img, bbox, camera_intrinsics, max_depth=1):
    x1, y1, x2, y2 = bbox
    H, W = depth_img.shape[:2]
    x1, x2 = max(0, x1), min(x2, W)
    y1, y2 = max(0, y1), min(y2, H)
    fx = camera_intrinsics[0,0]
    fy = camera_intrinsics[1,1]
    cx = camera_intrinsics[0,2]
    cy = camera_intrinsics[1,2]
    points_6d = []
    for v in range(y1, y2):
        for u in range(x1, x2):
            d_val = depth_img[v, u]
            if d_val == 0:
                continue
            Z = d_val / 1000.0
            if Z > max_depth:
                continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            b, g, r = color_img[v, u]
            R = r / 255.0
            G = g / 255.0
            B = b / 255.0
            points_6d.append([X, Y, Z, R, G, B])
    return np.array(points_6d, dtype=np.float32)

def depth_to_colored_pointcloud(depth_img, color_img, camera_intrinsics):
    H, W = depth_img.shape[:2]
    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]
    points = []
    colors = []
    for v in range(H):
        for u in range(W):
            z_val = depth_img[v, u]
            if z_val <= 0:
                continue
            Z = z_val / 1000.0
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            b, g, r = color_img[v, u]
            r /= 255.0; g /= 255.0; b /= 255.0
            points.append([X, Y, Z])
            colors.append([r, g, b])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors, dtype=np.float64))
    return pcd

def build_27_cubes_line_set(tomato_pose, neighbor_size, exclude_center=False, color=[0,1,0]):
    x, y, z = tomato_pose
    offsets = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [-1, 0, 1]:
                offsets.append((dx, dy, dz))
    half = neighbor_size / 2.0
    local_corners = [
        (-half, -half, -half),
        ( half, -half, -half),
        ( half,  half, -half),
        (-half,  half, -half),
        (-half, -half,  half),
        ( half, -half,  half),
        ( half,  half,  half),
        (-half,  half,  half)
    ]
    edges_idx = [(0,1), (1,2), (2,3), (3,0),
                 (4,5), (5,6), (6,7), (7,4),
                 (0,4), (1,5), (2,6), (3,7)]
    all_points = []
    all_lines = []
    for (dx, dy, dz) in offsets:
        nx = x + dx * neighbor_size
        ny = y + dy * neighbor_size
        nz = z + dz * neighbor_size
        start_idx = len(all_points)
        for (lx, ly, lz) in local_corners:
            all_points.append([nx + lx, ny + ly, nz + lz])
        for (i1, i2) in edges_idx:
            all_lines.append([start_idx + i1, start_idx + i2])
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(all_points, dtype=np.float64))
    line_set.lines = o3d.utility.Vector2iVector(np.array(all_lines, dtype=np.int32))
    line_colors = np.tile(np.array(color, dtype=np.float64), (len(all_lines), 1))
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    return line_set

def assign_cluster_colors(cluster_labels):
    unique_labels = np.unique(cluster_labels)
    color_dict = {}
    for label in unique_labels:
        if label == -1:
            continue
        if label in global_cluster_colors:
            color_dict[label] = global_cluster_colors[label]
        else:
            new_color = (np.random.randint(0, 255), np.random.randint(0, 255), np.random.randint(0, 255))
            global_cluster_colors[label] = new_color
            color_dict[label] = new_color
    return color_dict

def compute_distance_to_origin(point):
    dx = point[0] - 0       # x轴差值
    dy = point[1] - 0.24    # y轴差值
    dz = point[2] - 0       # z轴差值
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    return distance

def sphere_fit_on_cluster(points_6d, indices):
    cluster_xyz = points_6d[indices, 0:3]
    if len(cluster_xyz) < 4:
        return None
    res = fit_sphere_ransac(cluster_xyz, dist_threshold=0.01, max_iterations=1000, min_inliers=20)
    return res

def pick_largest_cluster(points_6d, labels):
    if len(labels) == 0:
        return None, []
    valid_mask = (labels != -1)
    valid_labels = labels[valid_mask]
    if len(valid_labels) == 0:
        return None, []
    unique_lbs, counts = np.unique(valid_labels, return_counts=True)
    best_label = unique_lbs[np.argmax(counts)]
    best_indices = np.where(labels == best_label)[0]
    return best_label, best_indices

def cluster_points_6d(points_6d, eps=0.01, min_samples=30):
    if len(points_6d) == 0:
        return np.array([])
    clusterer = DBSCAN(eps=eps, min_samples=min_samples, metric='euclidean')
    labels = clusterer.fit_predict(points_6d)
    return labels

# --------------------- YOLOv10ROS 类定义 ---------------------
class YOLOv10ROS:
    def __init__(self, weights_path):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = AutoBackend(weights=weights_path).to(self.device)
        self.names = self.model.names
        self.bridge = cv_bridge.CvBridge()

    def pixel_to_world(self, x, y, depth, camera_intrinsics):        
        # fx, fy = 433.394287109375, 432.917816162109
        # cx, cy = 324.0957641601562, 241.0320587158203
        fx, fy = 610.7744750976562, 610.5872802734375
        cx, cy = 322.503662109375, 249.54783630371094
        X = (x - cx) * depth/1000.0 / fx
        Y = (y - cy) * depth/1000.0 / fy
        Z = depth/1000.0
        return X, Y, Z

    def preprocess_letterbox(self, image):
        letterbox = LetterBox(new_shape=640, stride=32, auto=True)
        image = letterbox(image=image)
        image = (image[..., ::-1] / 255.0).astype(np.float32)
        image = image.transpose(2, 0, 1)[None]
        image = torch.from_numpy(image)
        return image

    def scale_coords_landmarks(self, img1_shape, coords, img0_shape, ratio_pad=None):
        if ratio_pad is None:
            gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
        else:
            gain = ratio_pad[0][0]
            pad = ratio_pad[1]
        coords[:, [0]] -= pad[0]
        coords[:, [1]] -= pad[1]
        coords[:, :10] /= gain
        coords[:, 0].clamp_(0, img0_shape[1])
        coords[:, 1].clamp_(0, img0_shape[0])
        return coords

    def show_results(self, img, xywh, conf, landmarks, class_num, is_plant):
        h, w, c = img.shape
        tl = 3
        x1 = int(xywh[0] * w - 0.5 * xywh[2] * w)
        y1 = int(xywh[1] * h - 0.5 * xywh[3] * h)
        x2 = int(xywh[0] * w + 0.5 * xywh[2] * w)
        y2 = int(xywh[1] * h + 0.5 * xywh[3] * h)
        if class_num == 0:
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), thickness=3, lineType=cv2.LINE_AA)
        else:
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), thickness=3, lineType=cv2.LINE_AA)
        return img

    def colorEncode(self, labelmap, colors, mode='RGB'):
        labelmap = labelmap.astype('int')
        labelmap_rgb = np.zeros((labelmap.shape[0], labelmap.shape[1], 3), dtype=np.uint8)
        aaaa = np.array([255, 0, 0])
        labelmap_rgb = labelmap_rgb + (labelmap == 1)[:, :, np.newaxis] * np.tile(aaaa, (labelmap.shape[0], labelmap.shape[1], 1))
        if mode == 'BGR':
            return labelmap_rgb[:, :, ::-1]
        else:
            return labelmap_rgb

    def detect_one(self, orgimg, depth_img, camera_intrinsics):
        img_size = 640
        conf_thres = 0.3
        iou_thres = 0.1
        img0 = copy.deepcopy(orgimg)
        imgx = copy.deepcopy(orgimg)
        h0, w0 = orgimg.shape[:2]
        r = img_size / max(h0, w0)
        if r != 1:
            interp = cv2.INTER_AREA if r < 1 else cv2.INTER_LINEAR
            img0 = cv2.resize(img0, (int(w0 * r), int(h0 * r)), interpolation=interp)
        imgsz = check_img_size(img_size, s=32)
        img = letterbox(img0, new_shape=imgsz)[0]
        gain = min(img.shape[0] / h0, img.shape[1] / w0)
        pad = (img.shape[1] - w0 * gain) / 2, (img.shape[0] - h0 * gain) / 2
        img = img[:, :, ::-1].transpose(2, 0, 1).copy()
        img = torch.from_numpy(img).to(self.device).float() / 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        pred = self.model(img)
        pred1 = ops.non_max_suppression(pred[0], conf_thres, iou_thres, labels=[], multi_label=True, agnostic=True, max_det=500, nc=2)
        
        pose_list = []
        for det in pred1:
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], orgimg.shape).round()
                det[:, 6:8] = self.scale_coords_landmarks(img.shape[2:], det[:, 6:8], orgimg.shape).round()
                for j in range(det.size()[0]):
                    xywh = (xyxy2xywh(det[j, :4].view(1, 4)) / torch.tensor([w0, h0, w0, h0]).to(self.device)).view(-1).tolist()
                    conf = det[j, 4].cpu().numpy()
                    is_plant = [det[j, 8] >= 0.5]
                    landmarks = (det[j, 6:8].view(1, 2) / torch.tensor([w0, h0]).to(self.device)).view(-1).tolist()
                    class_num = det[j, 5].cpu().numpy()
                    x1 = int(xywh[0] * w0 - 0.5 * xywh[2] * w0)
                    y1 = int(xywh[1] * h0 - 0.5 * xywh[3] * h0)
                    x2 = int(xywh[0] * w0 + 0.5 * xywh[2] * w0)
                    y2 = int(xywh[1] * h0 + 0.5 * xywh[3] * h0)
                    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w0, x2), min(h0, y2)
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    depth_value = depth_img[y_center, x_center]
                    # if depth_value == 0:
                    #     continue
                    X, Y, Z = self.pixel_to_world(x_center, y_center, depth_value, camera_intrinsics)
                    
                    print(X,Y,Z)
                    
                    
                    # Z = Z
                    # print(j, X, Y, Z)
                    # if Z > 0.6 :
                    #     continue  
                    pose_data = {
                        "bbox": [x1, y1, x2, y2],
                        "confidence": conf,
                        "class": int(class_num),
                        "3D_pose": (X, Y, Z)
                    }
                    imgx = self.show_results(imgx, xywh, conf, landmarks, class_num, is_plant)

                    pose_list.append(pose_data)
                    
        if pose_list:            
            pose_list.sort(key=lambda d: d["3D_pose"][2])
            selected_detection = pose_list[0]
            return imgx,selected_detection
        return imgx,None
            
class TomatoDetectionNode:
    def __init__(self):
        rospy.init_node("tomato_detection_node", anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.detector = YOLOv10ROS(weights_path="/home/mzc/jaka_robot/src/dianyunceshi/scripts/zuizhong/weights/best.pt")
        self.intrinsic_matrix = np.array([
            [916.1617431640625, 0.000000, 643.7554931640625],
            [0.000000, 915.8809204101562, 374.3217468261719],
            [0.0, 0.0, 1.0]
        ])
        self.pose_pub = rospy.Publisher("best_tomato_pose", tomato_poseGoal, queue_size=1)
        self.latest_color = None
        self.latest_depth = None
        
        # 分别订阅彩色图和深度图
        self.color_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1)
        
        # rospy.loginfo("TomatoDetectionNode initialized, waiting for images...")


    def color_callback(self, color_msg):
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error in color callback: %s", e)
            return
        self.latest_color = cv_color
        
        
    def depth_callback(self, depth_msg):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("CvBridge error in depth callback: %s", e)
            return
        self.latest_depth = cv_depth
        self.image_callback()



    def image_callback(self):
        if self.latest_color is None or self.latest_depth is None:
            return       

        imgx,goal = self.detector.detect_one(self.latest_color, self.latest_depth, self.intrinsic_matrix)
        if goal!=None:
            if goal["3D_pose"][2]>0.05:

                pose_msg = tomato_poseGoal()

                pose_msg.x = goal["3D_pose"][0]
                pose_msg.y = goal["3D_pose"][1]
                pose_msg.z = goal["3D_pose"][2]
                self.pose_pub.publish(pose_msg)
        
        cv2.imshow("Tomato Detection", imgx )
        cv2.waitKey(1)
        # rospy.sleep(0.01)

if __name__ == "__main__":
    try:
        node = TomatoDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass