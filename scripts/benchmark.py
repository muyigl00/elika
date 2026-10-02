"""Measure first and first-three-frame Shahed detections during a camera approach."""

from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any
import uuid

import yaml

from gazebo_service_client import GazeboServiceClient
from world_builder import WORLD_NAME, build_world


ROOT = Path(__file__).resolve().parents[1]
GAZEBO_MINIMUM_HFOV_DEG = math.degrees(0.1)


def finite_positive(value: object, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return number


def load_config(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Configuration root must be a mapping")
    for section in ("benchmark", "cameras", "detector", "target", "output"):
        if not isinstance(data.get(section), dict):
            raise ValueError(f"Missing configuration section: {section}")
    benchmark = data["benchmark"]
    speeds = [finite_positive(value, "speed") for value in benchmark["speeds_kmh"]]
    if len(speeds) != len(set(speeds)):
        raise ValueError("speeds_kmh must contain distinct values")
    benchmark["speeds_kmh"] = speeds
    for key in ("start_distance_m", "minimum_distance_m", "altitude_m", "sample_rate_hz"):
        benchmark[key] = finite_positive(benchmark[key], key)
    benchmark["required_consecutive_detections"] = int(
        benchmark["required_consecutive_detections"]
    )
    benchmark["maximum_frames_per_case"] = int(benchmark["maximum_frames_per_case"])
    if benchmark["required_consecutive_detections"] < 2:
        raise ValueError("required_consecutive_detections must be at least 2")
    if benchmark["maximum_frames_per_case"] < 1:
        raise ValueError("maximum_frames_per_case must be positive")
    if benchmark["minimum_distance_m"] >= benchmark["start_distance_m"]:
        raise ValueError("minimum_distance_m must be below start_distance_m")
    if benchmark.get("confirmation_policy") not in {"all_sensors", "any_sensor"}:
        raise ValueError("confirmation_policy must be all_sensors or any_sensor")

    cameras = data["cameras"]
    if set(cameras) != {"tele_rgb", "tele_thermal"}:
        raise ValueError("cameras must contain tele_rgb and tele_thermal")
    expected_modes = {"tele_rgb": "rgb", "tele_thermal": "pseudo_thermal"}
    for name, camera in cameras.items():
        if not isinstance(camera, dict):
            raise ValueError(f"Camera {name} must be a mapping")
        camera["width"] = int(camera["width"])
        camera["height"] = int(camera["height"])
        camera["fps"] = finite_positive(camera["fps"], f"{name}.fps")
        camera["horizontal_fov_deg"] = finite_positive(
            camera["horizontal_fov_deg"], f"{name}.horizontal_fov_deg"
        )
        camera["far_clip_m"] = finite_positive(
            camera["far_clip_m"], f"{name}.far_clip_m"
        )
        if camera["width"] < 320 or camera["height"] < 256:
            raise ValueError(f"Camera {name} resolution is too small")
        if not GAZEBO_MINIMUM_HFOV_DEG <= camera["horizontal_fov_deg"] < 120.0:
            raise ValueError(
                f"{name}.horizontal_fov_deg must be at least "
                f"{GAZEBO_MINIMUM_HFOV_DEG:.3f} and below 120"
            )
        if camera["far_clip_m"] <= benchmark["start_distance_m"]:
            raise ValueError(f"{name}.far_clip_m must exceed start_distance_m")
        if camera.get("mode") != expected_modes[name]:
            raise ValueError(f"{name}.mode must be {expected_modes[name]}")
        if not str(camera.get("profile", "")).strip():
            raise ValueError(f"{name}.profile is required")
    thermal = cameras["tele_thermal"]
    if thermal.get("transform") not in {"inverted_inferno", "grayscale"}:
        raise ValueError("tele_thermal.transform must be inverted_inferno or grayscale")
    thermal["pixel_pitch_um"] = finite_positive(
        thermal["pixel_pitch_um"], "tele_thermal.pixel_pitch_um"
    )

    detector = data["detector"]
    detector["confidence_threshold"] = finite_positive(
        detector["confidence_threshold"], "confidence_threshold"
    )
    detector["iou_threshold"] = finite_positive(detector["iou_threshold"], "iou_threshold")
    detector["inference_size"] = int(detector["inference_size"])
    detector["center_gate_fraction"] = finite_positive(
        detector["center_gate_fraction"], "center_gate_fraction"
    )
    if detector["confidence_threshold"] > 1.0 or detector["iou_threshold"] > 1.0:
        raise ValueError("Detection thresholds must be <= 1")
    if detector["center_gate_fraction"] > 0.5:
        raise ValueError("center_gate_fraction must be <= 0.5")
    for section, key in ((detector, "weights"), (data["target"], "model_sdf")):
        candidate = Path(str(section[key]))
        if not candidate.is_absolute():
            candidate = ROOT / candidate
        section[key] = str(candidate.resolve())
        if not candidate.is_file():
            raise FileNotFoundError(candidate)
    output = data["output"]
    output["save_video"] = bool(output.get("save_video", True))
    if not output["save_video"]:
        raise ValueError("save_video must stay true because all benchmark runs record video")
    output["video_fps"] = finite_positive(output["video_fps"], "video_fps")
    output["radar_width"] = int(output["radar_width"])
    output["radar_height"] = int(output["radar_height"])
    if output["radar_width"] < 640 or output["radar_height"] < 360:
        raise ValueError("Radar video must be at least 640x360")
    return data


def stop_processes(processes: list[subprocess.Popen]) -> None:
    for process in reversed(processes):
        if process.poll() is not None:
            continue
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)


class Video:
    def __init__(self, path: Path, fps: float, size: tuple[int, int]) -> None:
        import cv2

        self.writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size
        )
        if not self.writer.isOpened():
            raise RuntimeError(f"Could not open MP4 writer: {path}")

    def write(self, frame) -> None:
        self.writer.write(frame)

    def close(self) -> None:
        self.writer.release()


def pseudo_thermal(frame, transform: str):
    """Create a deterministic thermal-like image from a rendered RGB frame.

    This transformation is deliberately non-radiometric. It exercises a second
    visual-domain inference path without claiming physically accurate IR data.
    """
    import cv2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if transform == "grayscale":
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if transform == "inverted_inferno":
        return cv2.applyColorMap(255 - gray, cv2.COLORMAP_INFERNO)
    raise ValueError(f"Unknown pseudo-thermal transform: {transform}")


def render_radar(
    width: int,
    height: int,
    start_distance_m: float,
    observer_x_m: float,
    target_x_m: float,
    speed_kmh: float,
    flight_time_s: float,
    sensor_status: dict[str, dict[str, Any]],
    required_consecutive: int,
):
    """Render a simulator-ground-truth top-down overview with both objects."""
    import cv2
    import numpy as np

    frame = np.full((height, width, 3), (17, 24, 31), dtype=np.uint8)
    left, right = 90, width - 70
    center_y = height // 2
    usable = max(1.0, float(start_distance_m))

    def x_pixel(position_m: float) -> int:
        fraction = min(1.0, max(0.0, position_m / usable))
        return round(left + fraction * (right - left))

    cv2.putText(
        frame, "TOP-DOWN POSITION VIEW (SIMULATOR GROUND TRUTH)", (35, 45),
        cv2.FONT_HERSHEY_SIMPLEX, 0.83, (235, 240, 245), 2, cv2.LINE_AA,
    )
    cv2.line(frame, (left, center_y), (right, center_y), (95, 115, 128), 3, cv2.LINE_AA)
    tick_count = 10
    for index in range(tick_count + 1):
        x = round(left + index / tick_count * (right - left))
        cv2.line(frame, (x, center_y - 12), (x, center_y + 12), (120, 140, 152), 1)
        distance_km = start_distance_m * index / tick_count / 1000.0
        cv2.putText(
            frame, f"{distance_km:g} km", (x - 24, center_y + 42),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 185, 194), 1, cv2.LINE_AA,
        )

    observer_x = x_pixel(observer_x_m)
    target_x = x_pixel(target_x_m)
    cv2.arrowedLine(
        frame, (observer_x - 38, center_y - 55), (observer_x + 22, center_y - 55),
        (255, 190, 55), 4, cv2.LINE_AA, tipLength=0.25,
    )
    cv2.circle(frame, (observer_x, center_y), 13, (255, 190, 55), -1, cv2.LINE_AA)
    target_triangle = np.array(
        [[target_x, center_y - 18], [target_x - 17, center_y + 15],
         [target_x + 17, center_y + 15]], dtype=np.int32,
    )
    cv2.fillConvexPoly(frame, target_triangle, (65, 85, 255), cv2.LINE_AA)
    cv2.putText(
        frame, "OBSERVER", (max(12, observer_x - 48), center_y - 76),
        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 210, 95), 2, cv2.LINE_AA,
    )
    cv2.putText(
        frame, "STATIONARY TARGET", (max(12, target_x - 170), center_y - 38),
        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (105, 125, 255), 2, cv2.LINE_AA,
    )
    distance_m = max(0.0, target_x_m - observer_x_m)
    status_lines = [
        f"time={flight_time_s:.3f} s   speed={speed_kmh:g} km/h",
        f"observer_x={observer_x_m:.2f} m   target_x={target_x_m:.2f} m",
        f"separation={distance_m:.2f} m",
    ]
    for sensor_name in ("tele_rgb", "tele_thermal"):
        state = sensor_status.get(sensor_name, {})
        label = "RGB TELE" if sensor_name == "tele_rgb" else "THERMAL TELE"
        status_lines.append(
            f"{label}: detection={'YES' if state.get('detected') else 'NO'}  "
            f"streak={int(state.get('consecutive_count', 0))}/{required_consecutive}"
        )
    y = height - 145
    for index, line in enumerate(status_lines):
        color = (225, 232, 236)
        if line.startswith("RGB TELE:"):
            color = (100, 235, 130)
        elif line.startswith("THERMAL TELE:"):
            color = (90, 185, 255)
        cv2.putText(
            frame, line, (35, y + index * 27), cv2.FONT_HERSHEY_SIMPLEX,
            0.58, color, 1, cv2.LINE_AA,
        )
    return frame


def annotate_event(image, title: str, event: dict[str, Any], speed_kmh: float):
    import cv2

    output = image.copy()
    overlay = output.copy()
    cv2.rectangle(overlay, (0, 0), (output.shape[1], 155), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.76, output, 0.24, 0.0, output)
    bbox = event["bbox_xyxy"]
    lines = [
        title,
        f"distance={event['distance_m']:.2f} m  speed={speed_kmh:g} km/h  "
        f"confidence={event['confidence']:.3f}",
        f"flight_time={event['flight_time_s']:.3f} s  frame={event['frame_id']}",
        f"bbox=({bbox[0]:.1f}, {bbox[1]:.1f}, {bbox[2]:.1f}, {bbox[3]:.1f})  "
        f"size={bbox[2]-bbox[0]:.1f}x{bbox[3]-bbox[1]:.1f} px",
    ]
    for index, line in enumerate(lines):
        cv2.putText(
            output, line, (20, 34 + index * 32), cv2.FONT_HERSHEY_SIMPLEX,
            0.82 if index == 0 else 0.65,
            (0, 255, 255) if index == 0 else (255, 255, 255),
            2 if index == 0 else 1, cv2.LINE_AA,
        )
    return output


def draw_boxes(frame, boxes: list[dict[str, Any]], selected: dict[str, Any] | None):
    """Draw compact labels so a small target remains visible."""
    import cv2

    annotated = frame.copy()
    for box in boxes:
        x1, y1, x2, y2 = (round(value) for value in box["xyxy"])
        color = (0, 255, 0) if box is selected else (0, 200, 255)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        label = f"shahed {box['confidence']:.2f}"
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
        )
        label_y = max(text_height + 4, y1 - 5)
        cv2.rectangle(
            annotated,
            (x1, label_y - text_height - 4),
            (x1 + text_width + 4, label_y + baseline),
            (20, 20, 20),
            -1,
        )
        cv2.putText(
            annotated, label, (x1 + 2, label_y - 2), cv2.FONT_HERSHEY_SIMPLEX,
            0.48, color, 1, cv2.LINE_AA,
        )
    return annotated


def event_metadata(observation: dict[str, Any], box: dict[str, Any]) -> dict[str, Any]:
    return {
        "sensor": observation["sensor"],
        "frame_id": observation["frame_id"],
        "flight_time_s": observation["flight_time_s"],
        "distance_m": observation["distance_m"],
        "observer_x_m": observation["observer_x_m"],
        "target_x_m": observation["target_x_m"],
        "render_timestamp_s": observation["render_timestamp_s"],
        "confidence": box["confidence"],
        "bbox_xyxy": box["xyxy"],
        "bbox_width_px": box["xyxy"][2] - box["xyxy"][0],
        "bbox_height_px": box["xyxy"][3] - box["xyxy"][1],
    }


def flight_sample(
    frame_id: int, speed_kmh: float, sample_rate_hz: float, start_distance_m: float
) -> dict[str, float]:
    """Return exact kinematic state; the target position never changes."""
    flight_time = frame_id / sample_rate_hz
    observer_x = speed_kmh / 3.6 * flight_time
    return {
        "flight_time_s": flight_time,
        "observer_x_m": observer_x,
        "target_x_m": start_distance_m,
        "distance_m": start_distance_m - observer_x,
    }


def update_streak(streak: list, candidate, required: int) -> bool:
    """Update a consecutive-hit sequence and report confirmation."""
    if candidate is None:
        streak.clear()
        return False
    streak.append(candidate)
    if len(streak) > required:
        del streak[:-required]
    return len(streak) >= required


def save_event_pair(
    directory: Path,
    stem: str,
    title: str,
    candidate: dict[str, Any],
    speed_kmh: float,
) -> dict[str, Any]:
    import cv2

    raw_path = directory / f"{stem}_raw.png"
    yolo_path = directory / f"{stem}_yolo.png"
    evidence = annotate_event(candidate["annotated"], title, candidate["event"], speed_kmh)
    if not cv2.imwrite(str(raw_path), candidate["raw"]):
        raise RuntimeError(f"Failed to write {raw_path}")
    if not cv2.imwrite(str(yolo_path), evidence):
        raise RuntimeError(f"Failed to write {yolo_path}")
    saved = dict(candidate["event"])
    saved.update(raw_image=raw_path.name, yolo_image=yolo_path.name)
    return saved


def run_case(
    directory: Path,
    speed_kmh: float,
    config: dict,
    model,
    class_id: int,
    timeout_s: float,
) -> dict[str, Any]:
    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    benchmark = config["benchmark"]
    detector = config["detector"]
    output_config = config["output"]
    camera = build_world(directory / "world.sdf", config)
    (directory / "camera.json").write_text(json.dumps(camera, indent=2) + "\n")
    sensor_names = tuple(camera["sensors"])
    bridge_config = []
    for sensor in camera["sensors"].values():
        bridge_config.extend(
            [
                {
                    "ros_topic_name": sensor["image_topic"],
                    "gz_topic_name": sensor["image_topic"],
                    "ros_type_name": "sensor_msgs/msg/Image",
                    "gz_type_name": "gz.msgs.Image",
                    "direction": "GZ_TO_ROS",
                    "qos_profile": "SENSOR_DATA",
                },
                {
                    "ros_topic_name": sensor["camera_info_topic"],
                    "gz_topic_name": sensor["camera_info_topic"],
                    "ros_type_name": "sensor_msgs/msg/CameraInfo",
                    "gz_type_name": "gz.msgs.CameraInfo",
                    "direction": "GZ_TO_ROS",
                    "qos_profile": "SENSOR_DATA",
                },
            ]
        )
    bridge_path = directory / "bridge.yaml"
    bridge_path.write_text(yaml.safe_dump(bridge_config), encoding="utf-8")
    token = uuid.uuid4().hex
    domain_id = 80 + int(token[:6], 16) % 100
    environment = dict(
        os.environ,
        GZ_PARTITION="shahed_detection_" + token,
        ROS_DOMAIN_ID=str(domain_id),
    )
    processes: list[subprocess.Popen] = []
    client = None
    videos: dict[str, Video] = {}
    frames: dict[str, list[Image]] = {name: [] for name in sensor_names}
    camera_infos: dict[str, list[CameraInfo]] = {name: [] for name in sensor_names}
    states: dict[str, dict[str, Any]] = {
        name: {
            "first_detection": None,
            "streak": [],
            "confirmed_sequence": None,
            "inference_seconds": [],
            "frames_with_detection": 0,
        }
        for name in sensor_names
    }
    last_stamps = {name: -1.0 for name in sensor_names}
    final_observation = None
    final_radar = None
    reason = "unknown"

    rclpy.init(domain_id=domain_id)
    node = Node("shahed_detection_capture")
    converter = CvBridge()
    subscriptions = []
    for name, sensor in camera["sensors"].items():
        subscriptions.append(
            node.create_subscription(
                Image, sensor["image_topic"], frames[name].append, qos_profile_sensor_data
            )
        )
        subscriptions.append(
            node.create_subscription(
                CameraInfo,
                sensor["camera_info_topic"],
                camera_infos[name].append,
                qos_profile_sensor_data,
            )
        )

    def service(endpoint: str, kind: str, request: str) -> None:
        if client is None:
            raise RuntimeError("Gazebo service helper is unavailable")
        client.call(endpoint, kind, request)

    def fresh_image(sensor_name: str, after: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("Gazebo or ROS bridge exited; see gazebo.log")
            eligible = [
                message for message in frames[sensor_name]
                if message.header.stamp.sec + message.header.stamp.nanosec * 1e-9 > after + 1e-7
            ]
            if eligible:
                message = eligible[-1]
                frames[sensor_name].clear()
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                return stamp, converter.imgmsg_to_cv2(message, "bgr8")
            service("control", "WorldControl", "pause: true, multi_step: 40")
        raise RuntimeError(f"No fresh {sensor_name} camera frame; see gazebo.log")

    def drain_camera_queues() -> dict[str, float]:
        """Consume every frame already queued before changing the camera pose."""
        quiet_deadline = time.monotonic() + 0.05
        while time.monotonic() < quiet_deadline:
            count_before = sum(len(messages) for messages in frames.values())
            rclpy.spin_once(node, timeout_sec=0.005)
            if sum(len(messages) for messages in frames.values()) > count_before:
                quiet_deadline = time.monotonic() + 0.05
        for name, messages in frames.items():
            if messages:
                last_stamps[name] = max(
                    last_stamps[name],
                    max(
                        message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                        for message in messages
                    ),
                )
                messages.clear()
        return dict(last_stamps)

    def confirmation_reached() -> bool:
        confirmations = [
            states[name]["confirmed_sequence"] is not None for name in sensor_names
        ]
        if benchmark["confirmation_policy"] == "all_sensors":
            return all(confirmations)
        return any(confirmations)

    try:
        with (directory / "gazebo.log").open("w", encoding="utf-8") as gazebo_log, (
            directory / "frames.jsonl"
        ).open("w", encoding="utf-8") as records:
            try:
                subprocess.run(
                    ["bash", str(ROOT / "scripts/build_gazebo_service_client.sh")], check=True
                )
                client = GazeboServiceClient(
                    ROOT / ".cache/bin/gazebo_service_client", environment, gazebo_log
                )
                commands = (
                    ["gz", "sim", "-s", "--headless-rendering", str(directory / "world.sdf")],
                    [
                        "ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args", "-p",
                        "config_file:=" + str(bridge_path),
                    ],
                )
                for command in commands:
                    processes.append(
                        subprocess.Popen(
                            command, env=environment, stdout=gazebo_log, stderr=gazebo_log,
                            start_new_session=True,
                        )
                    )
                deadline = time.monotonic() + timeout_s
                required_service = f"/world/{WORLD_NAME}/set_pose"
                while required_service not in client.services():
                    if time.monotonic() >= deadline or any(
                        process.poll() is not None for process in processes
                    ):
                        raise RuntimeError("Gazebo startup failed; see gazebo.log")
                    time.sleep(0.1)

                startup_deadline = time.monotonic() + timeout_s
                while time.monotonic() < startup_deadline:
                    service("control", "WorldControl", "pause: true, multi_step: 40")
                    # CPU rendering of the 4K stream can need several seconds
                    # before DDS delivers all four initial subscriptions.
                    for _ in range(4):
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if all(frames[name] and camera_infos[name] for name in sensor_names):
                        for name in sensor_names:
                            last_stamps[name] = max(
                                message.header.stamp.sec
                                + message.header.stamp.nanosec * 1e-9
                                for message in frames[name]
                            )
                            frames[name].clear()
                        break
                else:
                    received = {
                        name: {
                            "images": len(frames[name]),
                            "camera_infos": len(camera_infos[name]),
                        }
                        for name in sensor_names
                    }
                    raise RuntimeError(
                        f"Both cameras did not produce startup frames: {received}"
                    )

                camera["verified_camera_matrices"] = {}
                for name in sensor_names:
                    sensor = camera["sensors"][name]
                    info = camera_infos[name][-1]
                    if (info.width, info.height) != (sensor["width"], sensor["height"]):
                        raise RuntimeError(
                            f"Unexpected {name} CameraInfo resolution: {info.width}x{info.height}"
                        )
                    if not math.isclose(
                        info.k[0], sensor["focal_length_px"], rel_tol=0.0, abs_tol=1.0
                    ):
                        raise RuntimeError(
                            f"{name} focal length mismatch: requested "
                            f"{sensor['focal_length_px']}, got {info.k[0]}"
                        )
                    camera["verified_camera_matrices"][name] = list(info.k)
                (directory / "camera.json").write_text(json.dumps(camera, indent=2) + "\n")

                for name in sensor_names:
                    sensor = camera["sensors"][name]
                    videos[name] = Video(
                        directory / f"{name}_yolo.mp4",
                        float(output_config["video_fps"]),
                        (sensor["width"], sensor["height"]),
                    )
                videos["radar"] = Video(
                    directory / "radar_topdown.mp4",
                    float(output_config["video_fps"]),
                    (int(output_config["radar_width"]), int(output_config["radar_height"])),
                )
                maximum_frames = int(benchmark["maximum_frames_per_case"])
                for frame_id in range(maximum_frames):
                    sample = flight_sample(
                        frame_id, speed_kmh, float(benchmark["sample_rate_hz"]),
                        float(benchmark["start_distance_m"]),
                    )
                    flight_time = sample["flight_time_s"]
                    observer_x = sample["observer_x_m"]
                    distance = sample["distance_m"]
                    if distance <= float(benchmark["minimum_distance_m"]):
                        reason = "minimum_distance_without_confirmation"
                        break
                    pose = (
                        f'name: "observer_camera", position: {{x: {observer_x}, y: 0, '
                        f'z: {float(benchmark["altitude_m"])}}}, '
                        "orientation: {x: 0, y: 0, z: 0, w: 1}"
                    )
                    pose_baseline_stamps = drain_camera_queues()
                    service("set_pose", "Pose", pose)
                    # At 30 Hz, 80 ms spans more than two camera periods. Requiring
                    # that timestamp prevents pre-pose DDS backlog from being used.
                    service("control", "WorldControl", "pause: true, multi_step: 80")
                    observation = {
                        "frame_id": frame_id,
                        "flight_time_s": flight_time,
                        "observer_x_m": observer_x,
                        "target_x_m": float(benchmark["start_distance_m"]),
                        "target_stationary": True,
                        "distance_m": distance,
                        "speed_kmh": speed_kmh,
                        "sensors": {},
                    }
                    required = int(benchmark["required_consecutive_detections"])
                    for name in sensor_names:
                        sensor = camera["sensors"][name]
                        stamp, rendered_frame = fresh_image(
                            name, pose_baseline_stamps[name] + 0.05
                        )
                        last_stamps[name] = stamp
                        if rendered_frame.shape[:2] != (
                            sensor["height"], sensor["width"]
                        ):
                            raise RuntimeError(
                                f"Unexpected {name} image shape: {rendered_frame.shape}"
                            )
                        frame = (
                            rendered_frame
                            if sensor["mode"] == "rgb"
                            else pseudo_thermal(
                                rendered_frame, str(sensor["transform"])
                            )
                        )
                        inference_started = time.perf_counter()
                        prediction = model.predict(
                            source=frame,
                            imgsz=int(detector["inference_size"]),
                            conf=float(detector["confidence_threshold"]),
                            iou=float(detector["iou_threshold"]),
                            classes=[class_id],
                            device=str(detector["device"]),
                            verbose=False,
                        )[0]
                        inference_seconds = time.perf_counter() - inference_started
                        states[name]["inference_seconds"].append(inference_seconds)
                        boxes = []
                        if prediction.boxes is not None:
                            boxes = [
                                {
                                    "xyxy": [float(value) for value in xyxy],
                                    "confidence": float(confidence),
                                }
                                for xyxy, confidence in zip(
                                    prediction.boxes.xyxy.tolist(),
                                    prediction.boxes.conf.tolist(),
                                )
                            ]
                        gate_x = sensor["width"] * float(
                            detector["center_gate_fraction"]
                        )
                        gate_y = sensor["height"] * float(
                            detector["center_gate_fraction"]
                        )
                        matches = [
                            box
                            for box in boxes
                            if abs(
                                (box["xyxy"][0] + box["xyxy"][2]) / 2.0
                                - sensor["width"] / 2.0
                            )
                            <= gate_x
                            and abs(
                                (box["xyxy"][1] + box["xyxy"][3]) / 2.0
                                - sensor["height"] / 2.0
                            )
                            <= gate_y
                        ]
                        selected = max(
                            matches, key=lambda box: box["confidence"], default=None
                        )
                        sensor_observation = {
                            "render_timestamp_s": stamp,
                            "detected": selected is not None,
                            "boxes": boxes,
                            "selected_box": selected,
                            "consecutive_count": 0,
                            "inference_seconds": inference_seconds,
                            "image_mode": (
                                sensor["mode"]
                                if sensor["mode"] == "rgb"
                                else f"pseudo_thermal:{sensor['transform']}"
                            ),
                            "horizontal_fov_deg": sensor["horizontal_fov_deg"],
                            "resolution": [sensor["width"], sensor["height"]],
                        }
                        event_observation = {
                            **observation,
                            "render_timestamp_s": stamp,
                            "sensor": name,
                        }
                        annotated = draw_boxes(frame, boxes, selected)
                        if selected is not None:
                            states[name]["frames_with_detection"] += 1
                            event = event_metadata(event_observation, selected)
                            candidate = {
                                "event": event,
                                "raw": frame.copy(),
                                "annotated": annotated.copy(),
                            }
                            if states[name]["first_detection"] is None:
                                states[name]["first_detection"] = save_event_pair(
                                    directory,
                                    f"{name}_first_detection",
                                    f"FIRST {name.upper()} SHAHED DETECTION",
                                    candidate,
                                    speed_kmh,
                                )
                                print(
                                    f"speed={speed_kmh:g} km/h sensor={name} "
                                    f"first_detection={distance:.2f} m "
                                    f"confidence={selected['confidence']:.3f}",
                                    flush=True,
                                )
                            newly_confirmed = update_streak(
                                states[name]["streak"], candidate, required
                            )
                        else:
                            newly_confirmed = update_streak(
                                states[name]["streak"], None, required
                            )
                        sensor_observation["consecutive_count"] = len(
                            states[name]["streak"]
                        )

                        if (
                            newly_confirmed
                            and states[name]["confirmed_sequence"] is None
                        ):
                            confirmed_sequence = []
                            for sequence_index, candidate in enumerate(
                                states[name]["streak"], start=1
                            ):
                                confirmed_sequence.append(
                                    save_event_pair(
                                        directory,
                                        f"{name}_first_three_consecutive_{sequence_index:02d}",
                                        f"{name.upper()} CONSECUTIVE SHAHED DETECTION "
                                        f"{sequence_index}/{required}",
                                        candidate,
                                        speed_kmh,
                                    )
                                )
                            states[name]["confirmed_sequence"] = confirmed_sequence
                            print(
                                f"speed={speed_kmh:g} km/h sensor={name} "
                                f"confirmed_at={distance:.2f} m frames="
                                f"{[item['frame_id'] for item in confirmed_sequence]}",
                                flush=True,
                            )

                        if bool(output_config["save_all_raw_frames"]):
                            raw_directory = directory / f"{name}_raw_frames"
                            raw_directory.mkdir(exist_ok=True)
                            cv2.imwrite(
                                str(raw_directory / f"frame_{frame_id:06d}.jpg"), frame
                            )
                        overlay_lines = [
                            f"{name.upper()}  speed={speed_kmh:g} km/h  "
                            f"distance={distance:.1f} m  frame={frame_id}",
                            f"shahed={'YES' if selected else 'NO'}  consecutive="
                            f"{len(states[name]['streak'])}/{required}",
                        ]
                        if sensor["mode"] == "pseudo_thermal":
                            overlay_lines.append(
                                "PSEUDO-THERMAL: PIPELINE TEST, NOT RADIOMETRIC IR"
                            )
                        start_y = sensor["height"] - 28 * len(overlay_lines) - 15
                        for line_index, line in enumerate(overlay_lines):
                            cv2.putText(
                                annotated,
                                line,
                                (18, start_y + line_index * 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.68,
                                (255, 255, 255),
                                2,
                                cv2.LINE_AA,
                            )
                        videos[name].write(annotated)
                        observation["sensors"][name] = sensor_observation

                    final_radar = render_radar(
                        int(output_config["radar_width"]),
                        int(output_config["radar_height"]),
                        float(benchmark["start_distance_m"]),
                        observer_x,
                        float(benchmark["start_distance_m"]),
                        speed_kmh,
                        flight_time,
                        observation["sensors"],
                        required,
                    )
                    videos["radar"].write(final_radar)
                    records.write(json.dumps(observation, separators=(",", ":")) + "\n")
                    records.flush()
                    final_observation = observation

                    if confirmation_reached():
                        reason = benchmark["confirmation_policy"] + "_confirmed"
                        break
                    if (frame_id + 1) % 100 == 0:
                        detected_text = ",".join(
                            f"{name}={observation['sensors'][name]['detected']}"
                            for name in sensor_names
                        )
                        print(
                            f"speed={speed_kmh:g} km/h frame={frame_id + 1} "
                            f"distance={distance:.1f} m detections={detected_text}", flush=True,
                        )
                else:
                    reason = "maximum_frames"
            finally:
                if client is not None:
                    client.close()
                    client = None
                stop_processes(processes)
    finally:
        for video in videos.values():
            video.close()
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()

    if final_radar is not None:
        radar_path = directory / "radar_final.png"
        if not cv2.imwrite(str(radar_path), final_radar):
            raise RuntimeError(f"Failed to write {radar_path}")

    sensor_results = {}
    for name in sensor_names:
        confirmed_sequence = states[name]["confirmed_sequence"] or []
        inference_times = states[name]["inference_seconds"]
        sensor = camera["sensors"][name]
        sensor_results[name] = {
            "image_mode": (
                sensor["mode"]
                if sensor["mode"] == "rgb"
                else f"pseudo_thermal:{sensor['transform']}"
            ),
            "profile": sensor["profile"],
            "resolution": [sensor["width"], sensor["height"]],
            "fps": sensor["fps"],
            "horizontal_fov_deg": sensor["horizontal_fov_deg"],
            "confirmed": bool(confirmed_sequence),
            "first_detection": states[name]["first_detection"],
            "first_three_consecutive_detections": confirmed_sequence,
            "confirmation_distance_m": (
                None if not confirmed_sequence else confirmed_sequence[-1]["distance_m"]
            ),
            "frames_with_detection": states[name]["frames_with_detection"],
            "mean_inference_ms": (
                None
                if not inference_times
                else 1000.0 * sum(inference_times) / len(inference_times)
            ),
            "video": f"{name}_yolo.mp4",
        }
    result = {
        "speed_kmh": speed_kmh,
        "target_stationary": True,
        "target_x_m": float(benchmark["start_distance_m"]),
        "confirmation_policy": benchmark["confirmation_policy"],
        "termination_reason": reason,
        "confirmed": confirmation_reached(),
        "sensors": sensor_results,
        "radar_video": "radar_topdown.mp4",
        "radar_final_image": "radar_final.png",
        "frames_processed": 0 if final_observation is None else final_observation["frame_id"] + 1,
        "end_distance_m": (
            float(benchmark["start_distance_m"])
            if final_observation is None else final_observation["distance_m"]
        ),
    }
    (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/benchmark.yaml")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--speeds-kmh", nargs="+", type=float)
    parser.add_argument(
        "--fovs-deg", nargs="+", type=float,
        help="Run a shared RGB/thermal horizontal-FOV sweep",
    )
    parser.add_argument("--start-distance-m", type=float)
    parser.add_argument("--device")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config.resolve())
    if args.speeds_kmh:
        config["benchmark"]["speeds_kmh"] = args.speeds_kmh
    if args.start_distance_m is not None:
        config["benchmark"]["start_distance_m"] = finite_positive(
            args.start_distance_m, "start_distance_m"
        )
        for camera_config in config["cameras"].values():
            camera_config["far_clip_m"] = max(
                camera_config["far_clip_m"], args.start_distance_m * 1.2
            )
    fov_values: list[float | None] = [None]
    if args.fovs_deg:
        fov_values = []
        for value in args.fovs_deg:
            fov = finite_positive(value, "fovs_deg")
            if not GAZEBO_MINIMUM_HFOV_DEG <= fov < 120.0:
                raise ValueError(
                    "Every sweep FOV must be at least "
                    f"{GAZEBO_MINIMUM_HFOV_DEG:.3f} and below 120 degrees"
                )
            fov_values.append(fov)
        if len(fov_values) != len(set(fov_values)):
            raise ValueError("fovs_deg must contain distinct values")
    if args.device:
        config["detector"]["device"] = args.device
    if args.smoke:
        config["benchmark"].update(
            start_distance_m=300.0,
            minimum_distance_m=20.0,
            speeds_kmh=[600.0],
            maximum_frames_per_case=100,
        )
        for camera_config in config["cameras"].values():
            camera_config["far_clip_m"] = 500.0
    if args.dry_run:
        print(yaml.safe_dump(config, sort_keys=False))
        return

    from ultralytics import YOLO

    model = YOLO(config["detector"]["weights"])
    expected_class = str(config["detector"]["class_name"])
    class_ids = [index for index, name in model.names.items() if name.casefold() == expected_class.casefold()]
    if len(class_ids) != 1:
        raise ValueError(f"Expected one '{expected_class}' class, got {model.names}")
    if 4 not in model.model.stride.tolist():
        raise ValueError("Long-range benchmark requires a P2/4 detector head")
    warmup_size = int(config["detector"]["inference_size"])
    import numpy as np

    model.predict(
        source=np.zeros((warmup_size, warmup_size, 3), dtype=np.uint8),
        imgsz=warmup_size,
        conf=float(config["detector"]["confidence_threshold"]),
        classes=class_ids,
        device=str(config["detector"]["device"]),
        verbose=False,
    )

    output = args.output or ROOT / "recordings" / (
        "run_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    )
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = {
        "status": "running",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "config_file": str(args.config.resolve()),
        "configuration": config,
        "model_names": model.names,
        "selected_class_id": class_ids[0],
        "detector_stride": model.model.stride.tolist(),
        "purpose": "first visual detection and first three consecutive detections",
        "sensor_modes": {
            "tele_rgb": "Gazebo RGB tele camera",
            "tele_thermal": (
                "deterministic pseudo-thermal tele camera; non-radiometric pipeline test"
            ),
        },
        "requested_fov_sweep_deg": [value for value in fov_values if value is not None],
        "videos_always_enabled": True,
        "radar_source": "simulator ground truth",
        "collision_detection": False,
        "target_motion": "stationary",
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    results = []
    try:
        for fov in fov_values:
            for speed_kmh in config["benchmark"]["speeds_kmh"]:
                case_config = copy.deepcopy(config)
                if fov is not None:
                    for camera_config in case_config["cameras"].values():
                        camera_config["horizontal_fov_deg"] = fov
                    directory_name = f"fov_{fov:g}deg_speed_{speed_kmh:g}kmh"
                else:
                    directory_name = f"speed_{speed_kmh:g}kmh"
                case_directory = output / directory_name
                case_directory.mkdir()
                active_fovs = {
                    name: camera_config["horizontal_fov_deg"]
                    for name, camera_config in case_config["cameras"].items()
                }
                print(
                    f"Starting stationary-target case at {speed_kmh:g} km/h "
                    f"with FOVs {active_fovs}",
                    flush=True,
                )
                result = run_case(
                    case_directory,
                    float(speed_kmh),
                    case_config,
                    model,
                    class_ids[0],
                    args.timeout,
                )
                result["directory"] = case_directory.name
                results.append(result)
        with (output / "summary.csv").open("w", encoding="utf-8", newline="") as stream:
            columns = [
                "speed_kmh", "tele_rgb_hfov_deg", "tele_thermal_hfov_deg",
                "confirmed", "termination_reason",
                "rgb_first_detection_distance_m", "rgb_confirmation_distance_m",
                "thermal_first_detection_distance_m", "thermal_confirmation_distance_m",
                "frames_processed", "end_distance_m", "directory",
            ]
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for result in results:
                writer.writerow({
                    "speed_kmh": result["speed_kmh"],
                    "tele_rgb_hfov_deg": result["sensors"]["tele_rgb"][
                        "horizontal_fov_deg"
                    ],
                    "tele_thermal_hfov_deg": result["sensors"]["tele_thermal"][
                        "horizontal_fov_deg"
                    ],
                    "confirmed": result["confirmed"],
                    "termination_reason": result["termination_reason"],
                    "rgb_first_detection_distance_m": (
                        None
                        if result["sensors"]["tele_rgb"]["first_detection"] is None
                        else result["sensors"]["tele_rgb"]["first_detection"]["distance_m"]
                    ),
                    "rgb_confirmation_distance_m": result["sensors"]["tele_rgb"][
                        "confirmation_distance_m"
                    ],
                    "thermal_first_detection_distance_m": (
                        None
                        if result["sensors"]["tele_thermal"]["first_detection"] is None
                        else result["sensors"]["tele_thermal"]["first_detection"][
                            "distance_m"
                        ]
                    ),
                    "thermal_confirmation_distance_m": result["sensors"]["tele_thermal"][
                        "confirmation_distance_m"
                    ],
                    "frames_processed": result["frames_processed"],
                    "end_distance_m": result["end_distance_m"],
                    "directory": result["directory"],
                })
        summary = {
            "status": "complete",
            "all_cases_confirmed": all(result["confirmed"] for result in results),
            "case_count": len(results),
            "results": results,
        }
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        manifest["status"] = "complete"
    except BaseException as error:
        manifest.update(status="failed", error=str(error))
        raise
    finally:
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Artifacts: {output}", flush=True)


if __name__ == "__main__":
    main()
