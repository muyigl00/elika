"""Create the isolated stationary-Shahed camera benchmark world."""

from __future__ import annotations

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_NAME = "shahed_detection"


def add(parent: ET.Element, tag: str, text: object | None = None, **attributes: str) -> ET.Element:
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def build_world(destination: Path, config: dict) -> dict:
    benchmark = config["benchmark"]
    camera_configs = config["cameras"]
    target_config = config["target"]

    sdf = ET.Element("sdf", version="1.10")
    world = add(sdf, "world", name=WORLD_NAME)
    physics = add(world, "physics", name="physics", type="ignored")
    add(physics, "max_step_size", "0.001")
    add(physics, "real_time_factor", "0")
    add(world, "gravity", "0 0 -9.81")
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        add(world, "plugin", filename=filename, name=name)
    sensors = add(
        world, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors"
    )
    add(sensors, "render_engine", "ogre2")
    scene = add(world, "scene")
    add(scene, "ambient", "0.66 0.66 0.66 1")
    add(scene, "background", "0.53 0.64 0.76 1")
    add(scene, "shadows", "true")
    light = add(world, "light", type="directional", name="sun")
    add(light, "pose", "0 0 1000 0 0 0")
    add(light, "diffuse", "0.92 0.90 0.86 1")
    add(light, "specular", "0.15 0.15 0.15 1")
    add(light, "direction", "-0.45 0.2 -1")

    ground = add(world, "model", name="flat_ground")
    add(ground, "static", "true")
    link = add(ground, "link", name="ground_link")
    visual = add(link, "visual", name="ground_visual")
    geometry = add(visual, "geometry")
    plane = add(geometry, "plane")
    add(plane, "normal", "0 0 1")
    add(plane, "size", "30000 30000")
    material = add(visual, "material")
    add(material, "ambient", "0.31 0.34 0.29 1")
    add(material, "diffuse", "0.34 0.37 0.31 1")

    model_path = Path(target_config["model_sdf"])
    target = copy.deepcopy(ET.parse(model_path).find("model"))
    if target is None:
        raise ValueError(f"No model element in {model_path}")
    target.set("name", "stationary_shahed")
    static = target.find("static")
    if static is None:
        static = add(target, "static")
    static.text = "true"
    roll, pitch, yaw = (
        math.radians(float(target_config[key]))
        for key in ("roll_deg", "pitch_deg", "yaw_deg")
    )
    pose = target.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        target.insert(1, pose)
    pose.text = (
        f"{float(benchmark['start_distance_m'])} 0 {float(benchmark['altitude_m'])} "
        f"{roll} {pitch} {yaw}"
    )
    world.append(target)

    observer = add(world, "model", name="observer_camera")
    add(observer, "static", "true")
    add(observer, "pose", f"0 0 {float(benchmark['altitude_m'])} 0 0 0")
    observer_link = add(observer, "link", name="camera_link")
    camera_sensors = {}

    def add_camera_sensor(name: str, camera_config: dict) -> None:
        width = int(camera_config["width"])
        height = int(camera_config["height"])
        hfov_deg = float(camera_config["horizontal_fov_deg"])
        hfov = math.radians(hfov_deg)
        focal = width / (2.0 * math.tan(hfov / 2.0))
        namespace = name
        image_topic = f"/shahed_benchmark/{namespace}/image_raw"
        info_topic = f"/shahed_benchmark/{namespace}/camera_info"
        sensor = add(observer_link, "sensor", name=f"{name}_camera", type="camera")
        add(sensor, "always_on", "true")
        add(sensor, "update_rate", float(camera_config["fps"]))
        add(sensor, "topic", image_topic)
        camera = add(sensor, "camera")
        add(camera, "horizontal_fov", hfov)
        add(camera, "camera_info_topic", info_topic)
        image = add(camera, "image")
        add(image, "width", width)
        add(image, "height", height)
        add(image, "format", "R8G8B8")
        clip = add(camera, "clip")
        add(clip, "near", "0.1")
        add(clip, "far", float(camera_config["far_clip_m"]))
        camera_sensors[namespace] = {
            "image_topic": image_topic,
            "camera_info_topic": info_topic,
            "width": width,
            "height": height,
            "fps": float(camera_config["fps"]),
            "horizontal_fov_deg": hfov_deg,
            "vertical_fov_deg": math.degrees(
                2.0 * math.atan(math.tan(hfov / 2.0) * height / width)
            ),
            "focal_length_px": focal,
            "mode": camera_config["mode"],
            "profile": camera_config["profile"],
        }
        if "transform" in camera_config:
            camera_sensors[namespace]["transform"] = camera_config["transform"]
        if "pixel_pitch_um" in camera_config:
            camera_sensors[namespace]["pixel_pitch_um"] = float(
                camera_config["pixel_pitch_um"]
            )

    for name, camera_config in camera_configs.items():
        add_camera_sensor(name, camera_config)

    ET.indent(sdf, space="  ")
    ET.ElementTree(sdf).write(destination, encoding="unicode", xml_declaration=True)
    return {
        "world_name": WORLD_NAME,
        "sensors": camera_sensors,
    }
