#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from box_detector_srv.srv import FindBoxCorners

import os
import sys
import json
from pathlib import Path
import yaml
import cv2
import torch
import numpy as np
import supervision as sv
from torchvision.ops import box_convert
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from grounding_dino.groundingdino.util.inference import load_model, load_image, predict
import open3d as o3d
from shapely.geometry import Polygon
from collections import defaultdict
import pyrealsense2 as rs

############################################################
# Setup paths and config
############################################################
script_dir = Path(__file__).resolve().parent
config_path = script_dir / 'config.yaml'
sys.path.append(str(script_dir))

if not config_path.exists():
    raise FileNotFoundError(f"Config file not found at {config_path}")

with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

# Use absolute path for output directory
default_output_path = (script_dir / "outputs1").resolve()
OUTPUT_DIR = Path(config.get('OUTPUT_DIR', str(default_output_path))).resolve()
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

############################################################
# Extract parameters
############################################################
TEXT_PROMPT = config.get('TEXT_PROMPT', "box.")
SAM2_CHECKPOINT = config.get('SAM2_CHECKPOINT', "./checkpoints/sam2.1_hiera_large.pt")
SAM2_MODEL_CONFIG = config.get('SAM2_MODEL_CONFIG', "configs/sam2.1/sam2.1_hiera_l.yaml")
GROUNDING_DINO_CONFIG = config.get('GROUNDING_DINO_CONFIG', "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py")
GROUNDING_DINO_CHECKPOINT = config.get('GROUNDING_DINO_CHECKPOINT', "gdino_checkpoints/groundingdino_swint_ogc.pth")
BOX_THRESHOLD = config.get('BOX_THRESHOLD', 0.2)
TEXT_THRESHOLD = config.get('TEXT_THRESHOLD', 0.25)
DEVICE = config.get('DEVICE', "cuda" if torch.cuda.is_available() else "cpu")

AREA_THRESHOLD = 300 * 300  # 90,000 pixels, for example
MEAN_DEPTH_THRESHOLD = config.get('MEAN_DEPTH_THRESHOLD', 2.0)  # e.g., 2.0 meters

############################################################
# Helper functions
############################################################

def count_pixels_in_polygon(mask, polygon, image_shape):
    polygon_mask = np.zeros(image_shape[:2], dtype=np.uint8)
    if not polygon.is_empty:
        pts = np.array(list(polygon.exterior.coords), dtype=np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.fillPoly(polygon_mask, [pts], 1)
    return np.count_nonzero(mask & (polygon_mask == 1))

def project_points_to_image(points_3d, fx, fy, px, py):
    X = points_3d[:, 0]
    Y = points_3d[:, 1]
    Z = points_3d[:, 2]
    u = (X * fx) / Z + px
    v = (Y * fy) / Z + py
    points_2d = np.stack([u, v], axis=1).astype(np.int32)
    return points_2d

def normal_from_plane_model(plane_model):
    n = np.array(plane_model[:3])
    return n / np.linalg.norm(n)

def vector_to_quaternion(normal):
    z_default = np.array([0,0,1], dtype=float)
    normal = normal / np.linalg.norm(normal)
    dot = np.dot(z_default, normal)
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot)
    if np.isclose(theta, 0.0, atol=1e-7):
        return np.array([1.0, 0.0, 0.0, 0.0])
    if np.isclose(theta, np.pi, atol=1e-7):
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = np.cross(z_default, normal)
    axis = axis / np.linalg.norm(axis)
    half_angle = theta / 2.0
    w = np.cos(half_angle)
    sin_half = np.sin(half_angle)
    x, y, z = axis * sin_half
    return np.array([w, x, y, z])

def quaternion_to_euler(quat):
    w, x, y, z = quat
    t0 = 2.0*(w*x + y*z)
    t1 = 1.0 - 2.0*(x*x + y*y)
    roll = np.arctan2(t0, t1)

    t2 = 2.0*(w*y - z*x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = 2.0*(w*z + x*y)
    t4 = 1.0 - 2.0*(y*y + z*z)
    yaw = np.arctan2(t3, t4)
    return roll, pitch, yaw

############################################################
# Main Server Node
############################################################

import pyrealsense2 as rs

class BoxCornerServer(Node):
    def __init__(self):
        super().__init__('box_corner_server')
        self.get_logger().info("Initializing camera...")
        self.pipeline, self.align, self.sensor = self.initialize_camera()

        self.srv = self.create_service(FindBoxCorners, 'find_box_corners', self.callback_find_box_corners)
        self.get_logger().info("Box Corner Server is ready.")

        # Pre-load models
        self.get_logger().info("Loading models...")
        self.grounding_model = load_model(
            model_config_path=GROUNDING_DINO_CONFIG,
            model_checkpoint_path=GROUNDING_DINO_CHECKPOINT,
            device=DEVICE
        )
        self.sam2_model = build_sam2(SAM2_MODEL_CONFIG, SAM2_CHECKPOINT, device=DEVICE)
        self.sam2_predictor = SAM2ImagePredictor(self.sam2_model)
        self.get_logger().info("Models and server initialized.")

    def initialize_camera(self):
        """Initialize and configure RealSense camera."""
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = pipeline.start(config)
        sensor = profile.get_device().first_color_sensor()
        # Enable auto exposure
        sensor.set_option(rs.option.enable_auto_exposure, 1)

        align_to = rs.stream.color
        align = rs.align(align_to)

        return pipeline, align, sensor

    def capture_frame_and_intrinsics(self):
        """Capture frames from the already initialized camera and extract intrinsics."""
        max_tries = 50
        frames_obtained = False
        for _ in range(max_tries):
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            if frames is not None:
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if depth_frame and color_frame:
                    frames_obtained = True
                    break
            self.get_logger().info("Waiting for frames...")

        if not frames_obtained:
            self.get_logger().error("Camera frames did not arrive within the timeout.")
            raise RuntimeError("Failed to capture frames from camera within given tries.")

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        color_path = os.path.join(str(OUTPUT_DIR), "color_image.jpg")
        depth_path = os.path.join(str(OUTPUT_DIR), "depth.npy")
        cv2.imwrite(color_path, color_image)
        np.save(depth_path, depth_image)

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        intrinsics_data = {
            "color": {
                "width": color_intrin.width,
                "height": color_intrin.height,
                "ppx": color_intrin.ppx,
                "ppy": color_intrin.ppy,
                "fx": color_intrin.fx,
                "fy": color_intrin.fy,
                "model": str(color_intrin.model),
                "coeffs": color_intrin.coeffs
            }
        }

        intrinsics_path = os.path.join(str(OUTPUT_DIR), "camera_intrinsics.json")
        with open(intrinsics_path, 'w') as f:
            json.dump(intrinsics_data, f, indent=4)

        return color_image, depth_image, intrinsics_data

    def run_pipeline(self):
        # 1) Capture frames
        self.get_logger().info("Capturing frame and intrinsics...")
        color_image, depth_image, intrinsics_data = self.capture_frame_and_intrinsics()

        fx = intrinsics_data["color"]["fx"]
        fy = intrinsics_data["color"]["fy"]
        px = intrinsics_data["color"]["ppx"]
        py = intrinsics_data["color"]["ppy"]

        # 2) Run GroundingDINO detection
        self.get_logger().info("Running GroundingDINO prediction...")
        color_path = os.path.join(str(OUTPUT_DIR), "color_image.jpg")
        image_source, image_tensor = load_image(color_path)
        boxes, confidences, labels = predict(
            model=self.grounding_model,
            image=image_tensor,
            caption=TEXT_PROMPT,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD,
        )

        if boxes.shape[0] == 0:
            return "ERROR: No boxes detected."

        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

        # 3) Run SAM2 segmentation
        self.get_logger().info("Running SAM2 prediction...")
        self.sam2_predictor.set_image(image_source)
        with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
            masks, scores, logits = self.sam2_predictor.predict(
                point_coords=None,
                point_labels=None,
                box=input_boxes,
                multimask_output=False,
            )

        if masks.ndim == 4:
            masks = masks.squeeze(1)
        if isinstance(masks, torch.Tensor):
            masks = masks.detach().cpu().numpy()
        masks = masks.astype(bool)

        confidences = confidences.numpy().tolist()
        class_names = labels
        class_ids = np.array(list(range(len(class_names))))

        # 4) Filter by area
        self.get_logger().info("Filtering masks by area and mean depth...")
        valid_indices = []
        for i, mask in enumerate(masks):
            area = np.count_nonzero(mask)
            if area > AREA_THRESHOLD:
                valid_indices.append(i)

        if len(valid_indices) == 0:
            return "ERROR: No masks passed area threshold."

        # 4.1) Filter by mean depth threshold
        depth_scale = 0.001  # or retrieve if needed from sensor
        final_indices = []
        for i in valid_indices:
            mask_i = masks[i]
            depth_values = depth_image[mask_i] * depth_scale
            depth_values = depth_values[depth_values > 0]  # remove invalid depth
            if len(depth_values) == 0:
                # No valid depth, skip or decide your logic
                continue
            mean_depth = np.mean(depth_values)
            # If mean depth >= threshold -> remove
            if mean_depth < MEAN_DEPTH_THRESHOLD:
                final_indices.append(i)

        if len(final_indices) == 0:
            return f"ERROR: All masks had mean depth >= {MEAN_DEPTH_THRESHOLD:.2f}, no valid masks remain."

        # Overwrite final
        masks = masks[final_indices]
        input_boxes = input_boxes[final_indices]
        confidences = [confidences[i] for i in final_indices]
        class_names = [class_names[i] for i in final_indices]
        class_ids = np.array(list(range(len(class_names))))

        detections = sv.Detections(
            xyxy=input_boxes,
            mask=masks,
            class_id=class_ids
        )

        # 5) Visualize GroundingDINO results
        box_annotator = sv.BoxAnnotator()
        label_annotator = sv.LabelAnnotator()
        labels_for_img = [f"{cn} {conf:.2f}" for cn, conf in zip(class_names, confidences)]
        img_with_boxes = color_image.copy()
        annotated_frame = box_annotator.annotate(scene=img_with_boxes, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections, labels=labels_for_img)
        cv2.imwrite(os.path.join(str(OUTPUT_DIR), "groundingdino_annotated_image.jpg"), annotated_frame)

        mask_annotator = sv.MaskAnnotator()
        annotated_frame_with_masks = mask_annotator.annotate(scene=annotated_frame.copy(), detections=detections)
        cv2.imwrite(os.path.join(str(OUTPUT_DIR), "grounded_sam2_annotated_image_with_mask.jpg"), annotated_frame_with_masks)

        # 6) Fit Rotated Rectangles
        rectangle_polygons = []
        for mask in masks:
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                rectangle_polygons.append(None)
                continue
            largest_contour = max(contours, key=cv2.contourArea)
            if largest_contour.shape[0] < 3:
                rectangle_polygons.append(None)
                continue

            rect = cv2.minAreaRect(largest_contour)
            box_points = cv2.boxPoints(rect)
            box_points = np.int64(box_points)
            poly = Polygon(box_points)
            if poly.is_empty or poly.area == 0:
                rectangle_polygons.append(None)
            else:
                rectangle_polygons.append(poly)

        rect_vis = color_image.copy()
        np.random.seed(42)
        rect_colors = [tuple(np.random.randint(0, 255, size=3).tolist()) for _ in rectangle_polygons]

        for i, poly in enumerate(rectangle_polygons):
            if poly is not None:
                pts = np.array(list(poly.exterior.coords), dtype=np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.polylines(rect_vis, [pts], isClosed=True, color=rect_colors[i], thickness=2)
                x, y = pts[0][0]
                cv2.putText(rect_vis, f"ID:{i}", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, rect_colors[i], 2)

        cv2.imwrite(os.path.join(str(OUTPUT_DIR), "rectangles_visualization.jpg"), rect_vis)

        # 7) Compute Intersections and Overlaps
        overlap_info = {}
        for i in range(len(rectangle_polygons)):
            if rectangle_polygons[i] is None:
                continue
            overlap_info[i] = {
                "overlap_count": 0,
                "overlaps_with": []
            }

        intersection_vis = color_image.copy()
        for i in range(len(rectangle_polygons)):
            if rectangle_polygons[i] is None:
                continue
            for j in range(i+1, len(rectangle_polygons)):
                if rectangle_polygons[j] is None:
                    continue
                p1 = rectangle_polygons[i]
                p2 = rectangle_polygons[j]
                inter = p1.intersection(p2)
                if not inter.is_empty:
                    overlap_info[i]["overlap_count"] += 1
                    overlap_info[i]["overlaps_with"].append(j)
                    overlap_info[j]["overlap_count"] += 1
                    overlap_info[j]["overlaps_with"].append(i)

                    if inter.geom_type == 'Polygon':
                        inter_polys = [inter]
                    elif inter.geom_type == 'MultiPolygon':
                        inter_polys = list(inter.geoms)
                    else:
                        inter_polys = []

                    overlay = np.zeros_like(intersection_vis, dtype=intersection_vis.dtype)
                    color = tuple(np.random.randint(0, 255, size=3).tolist())
                    for ipoly in inter_polys:
                        ipts = np.array(list(ipoly.exterior.coords), dtype=np.int32)
                        ipts = ipts.reshape((-1,1,2))
                        cv2.fillPoly(overlay, [ipts], color)

                    alpha = 0.5
                    intersection_vis = cv2.addWeighted(intersection_vis, 1.0, overlay, alpha, 0)

        cv2.imwrite(os.path.join(str(OUTPUT_DIR), "intersections_rectangles_visualization.jpg"), intersection_vis)

        boxes_by_overlap = defaultdict(list)
        for i, info in overlap_info.items():
            boxes_by_overlap[info["overlap_count"]].append(i)

        sorted_overlap_counts = sorted(boxes_by_overlap.keys())
        chosen_box = None

        for oc in sorted_overlap_counts:
            candidates = boxes_by_overlap[oc]
            if oc == 0:
                chosen_box = candidates[0]
                break
            else:
                for candidate in candidates:
                    overlaps_with = overlap_info[candidate]["overlaps_with"]
                    all_on_top = True
                    for other in overlaps_with:
                        p1 = rectangle_polygons[candidate]
                        p2 = rectangle_polygons[other]
                        if p1 is None or p2 is None:
                            all_on_top = False
                            break
                        inter = p1.intersection(p2)
                        if inter.is_empty:
                            all_on_top = False
                            break
                        candidate_count = count_pixels_in_polygon(masks[candidate], inter, color_image.shape)
                        other_count = count_pixels_in_polygon(masks[other], inter, color_image.shape)
                        if candidate_count <= other_count:
                            all_on_top = False
                            break
                    if all_on_top:
                        chosen_box = candidate
                        break
                if chosen_box is not None:
                    break

        if chosen_box is None:
            if len(masks) > 0:
                chosen_box = 0
            else:
                return "ERROR: No suitable box found."

        chosen_mask = masks[chosen_box]

        # 8) Build point cloud for chosen mask
        # (Assuming scale of 0.001 = 1mm -> 1m if using RealSense default)
        depth_scale = 0.001
        points = []
        for v in range(color_image.shape[0]):
            for u in range(color_image.shape[1]):
                if chosen_mask[v, u]:
                    Z = depth_image[v, u] * depth_scale
                    if Z > 0:
                        X = (u - px) * Z / fx
                        Y = (v - py) * Z / fy
                        points.append([X, Y, Z])

        points = np.array(points)
        if len(points) == 0:
            return "ERROR: No points found in chosen mask."

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 9) Pre-processing for plane segmentation
        pcd_center = pcd.get_center()
        pcd.translate(-pcd_center)
        nn = 16
        std_multiplier = 10
        _, ind = pcd.remove_statistical_outlier(nn, std_multiplier)
        pcd_filtered = pcd.select_by_index(ind)
        voxel_size = 0.005
        pcd_downsampled = pcd_filtered.voxel_down_sample(voxel_size=voxel_size)
        nn_distance = np.mean(np.asarray(pcd_downsampled.compute_nearest_neighbor_distance()))
        radius_normals = nn_distance * 4
        pcd_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normals, max_nn=16), fast_normal_computation=True)
        pcd_downsampled.orient_normals_towards_camera_location(np.array([0,0,0]))

        # 10) Multi-plane segmentation
        max_plane_idx = 6
        pt_to_plane_dist = 0.01
        segment_models = {}
        segments = {}
        rest = pcd_downsampled

        for i in range(max_plane_idx):
            if len(rest.points) < 50:
                break
            plane_model, inliers = rest.segment_plane(distance_threshold=pt_to_plane_dist, ransac_n=3, num_iterations=1000)
            if len(inliers) == 0:
                break
            segment_models[i] = plane_model
            segments[i] = rest.select_by_index(inliers)
            rest = rest.select_by_index(inliers, invert=True)

        if len(segments) == 0:
            return "ERROR: No planes detected in chosen box."

        # 11) Choose optimal plane
        up_vector = np.array([0,0,1])
        best_plane_idx = None
        best_score = -np.inf
        for i, plane_model in segment_models.items():
            n = normal_from_plane_model(plane_model)
            dot_with_up = np.dot(n, up_vector)
            plane_size = len(segments[i].points)
            score = dot_with_up * 0.7 + (plane_size / 1000.0) * 0.3
            if score > best_score:
                best_score = score
                best_plane_idx = i

        chosen_plane = segments[best_plane_idx]
        plane_points = np.asarray(chosen_plane.points)
        # Translate back
        plane_points += pcd_center

        # Suction point = centroid
        centroid = np.mean(plane_points, axis=0)

        # Orientation:
        chosen_plane_normal = normal_from_plane_model(segment_models[best_plane_idx])
        quat = vector_to_quaternion(chosen_plane_normal)
        roll, pitch, yaw = quaternion_to_euler(quat)

        # 12) Visualize chosen plane on the image
        if len(plane_points) > 0:
            plane_points_2d = project_points_to_image(plane_points, fx, fy, px, py)
            chosen_plane_vis = color_image.copy()
            # Draw plane points
            for pt in plane_points_2d:
                u,v = pt
                if 0 <= u < chosen_plane_vis.shape[1] and 0 <= v < chosen_plane_vis.shape[0]:
                    chosen_plane_vis[v,u] = (0,255,0)

            # Mark suction point
            suction_2d = project_points_to_image(centroid[None,:], fx, fy, px, py)[0]
            cv2.circle(chosen_plane_vis, (suction_2d[0], suction_2d[1]), 5, (0,0,255), -1)
            cv2.imwrite(os.path.join(str(OUTPUT_DIR), "chosen_plane_visualization.jpg"), chosen_plane_vis)

        quat_str = f"{quat[0]},{quat[1]},{quat[2]},{quat[3]}"
        response_str = f"Position: {centroid[0]:.4f},{centroid[1]:.4f},{centroid[2]:.4f}; " \
                       f"RPY: {roll:.4f},{pitch:.4f},{yaw:.4f}; Quaternion: {quat_str}"

        return response_str

    def callback_find_box_corners(self, request, response):
        if request.request == "MARCO":
            self.get_logger().info("Received MARCO request. Running pipeline...")
            try:
                result = self.run_pipeline()
                response.response = result
            except Exception as e:
                self.get_logger().error(f"Error during run_pipeline: {e}")
                response.response = "ERROR: " + str(e)
        else:
            response.response = "ERROR: Unsupported request."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BoxCornerServer()
    node.get_logger().info("Box Detector Server is running and waiting for requests...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
