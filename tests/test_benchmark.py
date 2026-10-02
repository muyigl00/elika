from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from benchmark import flight_sample, load_config, pseudo_thermal, render_radar, update_streak
from world_builder import build_world


def test_four_requested_speeds_and_stationary_target(tmp_path):
    config = load_config(ROOT / "configs/benchmark.yaml")
    assert config["benchmark"]["speeds_kmh"] == [600.0, 800.0, 1000.0, 1200.0]
    for speed in config["benchmark"]["speeds_kmh"]:
        start = config["benchmark"]["start_distance_m"]
        first = flight_sample(0, speed, 30.0, start)
        later = flight_sample(30, speed, 30.0, start)
        assert first["target_x_m"] == later["target_x_m"] == start
        assert later["observer_x_m"] == speed / 3.6
        assert later["distance_m"] == start - speed / 3.6


def test_three_consecutive_hits_and_reset():
    streak = []
    assert not update_streak(streak, {"frame": 1}, 3)
    assert not update_streak(streak, {"frame": 2}, 3)
    assert not update_streak(streak, None, 3)
    assert streak == []
    assert not update_streak(streak, {"frame": 4}, 3)
    assert not update_streak(streak, {"frame": 5}, 3)
    assert update_streak(streak, {"frame": 6}, 3)
    assert [item["frame"] for item in streak] == [4, 5, 6]


def test_generated_world_has_static_target_and_no_contact_sensor(tmp_path):
    config = load_config(ROOT / "configs/benchmark.yaml")
    path = tmp_path / "world.sdf"
    camera = build_world(path, config)
    root = ET.parse(path)
    target = root.find(".//model[@name='stationary_shahed']")
    assert target is not None
    assert target.findtext("static") == "true"
    assert float(target.findtext("pose").split()[0]) == 10000.0
    assert not root.findall(".//sensor[@type='contact']")
    assert set(camera["sensors"]) == {"tele_rgb", "tele_thermal"}
    assert camera["sensors"]["tele_rgb"]["width"] == 3840
    assert camera["sensors"]["tele_rgb"]["height"] == 2160
    assert camera["sensors"]["tele_rgb"]["horizontal_fov_deg"] == 18.0
    assert camera["sensors"]["tele_thermal"]["width"] == 640
    assert camera["sensors"]["tele_thermal"]["height"] == 512
    assert camera["sensors"]["tele_thermal"]["pixel_pitch_um"] == 12.0
    sensors = root.findall(".//model[@name='observer_camera']//sensor[@type='camera']")
    assert {sensor.get("name") for sensor in sensors} == {
        "tele_rgb_camera", "tele_thermal_camera"
    }
    by_name = {sensor.get("name"): sensor for sensor in sensors}
    assert by_name["tele_rgb_camera"].findtext("camera/image/width") == "3840"
    assert by_name["tele_rgb_camera"].findtext("camera/image/height") == "2160"
    assert by_name["tele_thermal_camera"].findtext("camera/image/width") == "640"
    assert by_name["tele_thermal_camera"].findtext("camera/image/height") == "512"


def test_pseudo_thermal_and_radar_outputs_have_requested_shapes():
    import numpy as np

    image = np.zeros((108, 192, 3), dtype=np.uint8)
    image[:, 96:] = 255
    thermal = pseudo_thermal(image, "inverted_inferno")
    assert thermal.shape == image.shape
    assert not np.array_equal(thermal, image)
    radar = render_radar(
        640, 360, 10000.0, 2500.0, 10000.0, 600.0, 45.0,
        {"tele_rgb": {"detected": True, "consecutive_count": 2},
         "tele_thermal": {"detected": False, "consecutive_count": 0}},
        3,
    )
    assert radar.shape == (360, 640, 3)
