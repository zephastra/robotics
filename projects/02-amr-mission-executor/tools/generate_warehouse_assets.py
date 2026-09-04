#!/usr/bin/env python3
"""Generate the Project 002 Gazebo world and matching occupancy map."""

from dataclasses import dataclass
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
RESOLUTION = 0.05
ORIGIN_X = -14.0
ORIGIN_Y = -12.0
WIDTH = 560
HEIGHT = 480


@dataclass(frozen=True)
class Rectangle:
    name: str
    x: float
    y: float
    size_x: float
    size_y: float
    height: float
    color: tuple[float, float, float]


WALL_COLOR = (0.55, 0.58, 0.64)
SHELF_COLOR = (0.16, 0.33, 0.52)

FIXED_OBSTACLES = [
    Rectangle("north_wall", 0.0, 11.0, 26.3, 0.3, 1.8, WALL_COLOR),
    Rectangle("south_wall", 0.0, -11.0, 26.3, 0.3, 1.8, WALL_COLOR),
    Rectangle("west_wall", -13.0, 0.0, 0.3, 22.3, 1.8, WALL_COLOR),
    Rectangle("east_wall", 13.0, 0.0, 0.3, 22.3, 1.8, WALL_COLOR),
    Rectangle("shelf_a1", -7.0, 4.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_a2", -3.0, 4.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_b1", 1.0, 4.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_b2", 5.0, 4.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_c1", -7.0, -3.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_c2", -3.0, -3.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_d1", 1.0, -3.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("shelf_d2", 5.0, -3.5, 1.0, 5.2, 1.7, SHELF_COLOR),
    Rectangle("inbound_dock", -10.5, 8.5, 3.0, 0.8, 0.7, (0.20, 0.65, 0.28)),
    Rectangle("outbound_dock", 10.5, 8.5, 3.0, 0.8, 0.7, (0.90, 0.48, 0.12)),
    Rectangle("quality_station", 10.0, -8.0, 2.6, 0.9, 0.9, (0.78, 0.68, 0.10)),
    Rectangle("charging_dock", -12.2, -9.0, 0.4, 2.0, 0.8, (0.48, 0.22, 0.72)),
]

FLOOR_ZONES = [
    Rectangle("inbound_zone", -10.5, 7.0, 3.0, 1.6, 0.01, (0.25, 0.72, 0.32)),
    Rectangle("outbound_zone", 10.5, 7.0, 3.0, 1.6, 0.01, (0.95, 0.54, 0.12)),
    Rectangle("quality_zone", 10.0, -6.5, 3.0, 1.5, 0.01, (0.88, 0.78, 0.18)),
    Rectangle("charging_zone", -11.0, -9.0, 1.8, 2.4, 0.01, (0.58, 0.30, 0.82)),
]


def safe_name(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


def visual_material(color: tuple[float, float, float]) -> str:
    red, green, blue = color
    return (
        f"<material><ambient>{red} {green} {blue} 1</ambient>"
        f"<diffuse>{red} {green} {blue} 1</diffuse></material>"
    )


def fixed_link(rectangle: Rectangle) -> str:
    name = safe_name(rectangle.name)
    pose_z = rectangle.height / 2.0
    size = f"{rectangle.size_x} {rectangle.size_y} {rectangle.height}"
    material = visual_material(rectangle.color)
    return f"""
      <link name="{name}">
        <pose>{rectangle.x} {rectangle.y} {pose_z} 0 0 0</pose>
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{size}</size></box></geometry>{material}</visual>
      </link>"""


def zone_visual(rectangle: Rectangle) -> str:
    name = safe_name(rectangle.name)
    size = f"{rectangle.size_x} {rectangle.size_y} {rectangle.height}"
    material = visual_material(rectangle.color)
    return f"""
      <link name="{name}">
        <pose>{rectangle.x} {rectangle.y} 0.006 0 0 0</pose>
        <visual name="visual"><geometry><box><size>{size}</size></box></geometry>{material}</visual>
      </link>"""


def robot_sdf() -> str:
    return """
    <model name="warehouse_amr">
      <pose>-11 -9 0 0 0 3.141592653589793</pose>
      <link name="base_link">
        <inertial>
          <pose>-0.04 0 0.13 0 0 0</pose><mass>9.0</mass>
          <inertia><ixx>0.12</ixx><iyy>0.11</iyy><izz>0.16</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <pose>0 0 0.15 0 0 0</pose><geometry><box><size>0.55 0.38 0.18</size></box></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <pose>0 0 0.15 0 0 0</pose><geometry><box><size>0.55 0.38 0.18</size></box></geometry>
          <material><ambient>0.08 0.34 0.78 1</ambient><diffuse>0.08 0.34 0.78 1</diffuse></material>
        </visual>
      </link>
      <link name="left_wheel">
        <pose relative_to="base_link">0 0.21 0.08 0 0 0</pose>
        <inertial><mass>0.35</mass><inertia><ixx>0.0008</ixx><iyy>0.0008</iyy><izz>0.0008</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>0.08</radius><length>0.04</length></cylinder></geometry><surface><friction><ode><mu>1.3</mu><mu2>1.3</mu2></ode></friction></surface></collision>
        <visual name="visual"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>0.08</radius><length>0.04</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
      </link>
      <link name="right_wheel">
        <pose relative_to="base_link">0 -0.21 0.08 0 0 0</pose>
        <inertial><mass>0.35</mass><inertia><ixx>0.0008</ixx><iyy>0.0008</iyy><izz>0.0008</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>0.08</radius><length>0.04</length></cylinder></geometry><surface><friction><ode><mu>1.3</mu><mu2>1.3</mu2></ode></friction></surface></collision>
        <visual name="visual"><pose>0 0 0 1.5707963 0 0</pose><geometry><cylinder><radius>0.08</radius><length>0.04</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
      </link>
      <link name="caster">
        <pose relative_to="base_link">-0.21 0 0.035 0 0 0</pose>
        <inertial><mass>0.08</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>0.035</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>0.035</radius></sphere></geometry><material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
      </link>
      <link name="lidar_link">
        <pose relative_to="base_link">0 0 0.24 0 0 0</pose>
        <inertial><mass>0.08</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <visual name="visual"><geometry><cylinder><radius>0.045</radius><length>0.035</length></cylinder></geometry><material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
        <sensor type="gpu_lidar" name="warehouse_lidar">
          <pose>0 0 0.025 0 0 0</pose><topic>/warehouse/scan</topic><update_rate>12</update_rate><always_on>1</always_on><visualize>true</visualize>
          <lidar><scan><horizontal><samples>720</samples><resolution>1</resolution><min_angle>-3.14159265358979</min_angle><max_angle>3.14159265358979</max_angle></horizontal><vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan><range><min>0.15</min><max>20.0</max></range></lidar>
        </sensor>
      </link>
      <joint name="left_wheel_joint" type="revolute"><parent>base_link</parent><child>left_wheel</child><pose relative_to="left_wheel">0 0 0 0 0 0</pose><axis><xyz expressed_in="base_link">0 1 0</xyz></axis></joint>
      <joint name="right_wheel_joint" type="revolute"><parent>base_link</parent><child>right_wheel</child><pose relative_to="right_wheel">0 0 0 0 0 0</pose><axis><xyz expressed_in="base_link">0 1 0</xyz></axis></joint>
      <joint name="caster_joint" type="fixed"><parent>base_link</parent><child>caster</child><pose relative_to="caster">0 0 0 0 0 0</pose></joint>
      <joint name="lidar_joint" type="fixed"><parent>base_link</parent><child>lidar_link</child><pose relative_to="lidar_link">0 0 0 0 0 0</pose></joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.42</wheel_separation><wheel_radius>0.08</wheel_radius>
        <odom_publish_frequency>30</odom_publish_frequency><topic>/model/warehouse_amr/cmd_vel</topic><odom_topic>/model/warehouse_amr/odometry</odom_topic><publish_odom>true</publish_odom><publish_wheel_tf>false</publish_wheel_tf>
      </plugin>
    </model>"""


def movable_cart_sdf() -> str:
    return """
    <model name="movable_test_cart">
      <static>true</static>
      <pose>10 0 0.45 0 0 0</pose>
      <link name="body">
        <inertial><mass>18.0</mass><inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="collision"><geometry><box><size>2.4 0.7 0.9</size></box></geometry><surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><box><size>2.4 0.7 0.9</size></box></geometry><material><ambient>0.75 0.18 0.14 1</ambient><diffuse>0.75 0.18 0.14 1</diffuse></material></visual>
      </link>
    </model>"""


def generate_world() -> str:
    obstacle_links = "".join(fixed_link(item) for item in FIXED_OBSTACLES)
    zone_links = "".join(zone_visual(item) for item in FLOOR_ZONES)
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="warehouse_mission_world">
    <physics name="default" type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <model name="warehouse_floor"><static>true</static><link name="floor"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>32 28</size></plane></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>32 28</size></plane></geometry><material><ambient>0.70 0.70 0.70 1</ambient><diffuse>0.70 0.70 0.70 1</diffuse></material></visual></link></model>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 14 0 0 0</pose><diffuse>0.85 0.85 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
    <model name="warehouse_structure"><static>true</static>{obstacle_links}</model>
    <model name="warehouse_floor_zones"><static>true</static>{zone_links}</model>
{movable_cart_sdf()}
{robot_sdf()}
  </world>
</sdf>
"""


def world_to_pixel(x: float, y: float) -> tuple[int, int]:
    column = int((x - ORIGIN_X) / RESOLUTION)
    row_from_bottom = int((y - ORIGIN_Y) / RESOLUTION)
    return column, HEIGHT - 1 - row_from_bottom


def fill_rectangle(
    pixels: bytearray,
    x: float,
    y: float,
    size_x: float,
    size_y: float,
    value: int,
) -> None:
    min_column, max_row = world_to_pixel(x - size_x / 2.0, y - size_y / 2.0)
    max_column, min_row = world_to_pixel(x + size_x / 2.0, y + size_y / 2.0)
    min_column = max(0, min(WIDTH - 1, min_column))
    max_column = max(0, min(WIDTH - 1, max_column))
    min_row = max(0, min(HEIGHT - 1, min_row))
    max_row = max(0, min(HEIGHT - 1, max_row))
    for row in range(min_row, max_row + 1):
        start = row * WIDTH + min_column
        end = row * WIDTH + max_column + 1
        pixels[start:end] = bytes([value]) * (end - start)


def generate_map() -> bytes:
    pixels = bytearray([205]) * (WIDTH * HEIGHT)
    fill_rectangle(pixels, 0.0, 0.0, 25.7, 21.7, 254)
    for obstacle in FIXED_OBSTACLES:
        fill_rectangle(
            pixels,
            obstacle.x,
            obstacle.y,
            obstacle.size_x,
            obstacle.size_y,
            0,
        )
    header = f"P5\n# Project 002 warehouse occupancy map\n{WIDTH} {HEIGHT}\n255\n"
    return header.encode("ascii") + pixels


def main() -> None:
    worlds_directory = ROOT / "worlds"
    maps_directory = ROOT / "maps"
    worlds_directory.mkdir(parents=True, exist_ok=True)
    maps_directory.mkdir(parents=True, exist_ok=True)

    world_path = worlds_directory / "warehouse_world.sdf"
    map_path = maps_directory / "warehouse_map.pgm"
    yaml_path = maps_directory / "warehouse_map.yaml"

    world_path.write_text(generate_world(), encoding="utf-8")
    map_path.write_bytes(generate_map())
    yaml_path.write_text(
        "image: warehouse_map.pgm\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION:.3f}\n"
        f"origin: [{ORIGIN_X:.3f}, {ORIGIN_Y:.3f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    print(f"Generated {world_path}")
    print(f"Generated {map_path} ({WIDTH} x {HEIGHT})")
    print(f"Generated {yaml_path}")


if __name__ == "__main__":
    main()
