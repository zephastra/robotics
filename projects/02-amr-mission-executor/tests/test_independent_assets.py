from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    with path.open("rb") as stream:
        assert stream.readline().strip() == b"P5"
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = (int(value) for value in line.split())
        assert stream.readline().strip() == b"255"
        pixels = stream.read()
    assert len(pixels) == width * height
    return width, height, pixels


def test_world_and_map_assets_exist_and_parse() -> None:
    world_path = PROJECT_ROOT / "worlds" / "warehouse_world.sdf"
    map_yaml_path = PROJECT_ROOT / "maps" / "warehouse_map.yaml"
    map_image_path = PROJECT_ROOT / "maps" / "warehouse_map.pgm"

    root = element_tree.parse(world_path).getroot()
    world = root.find("world")
    assert world is not None
    assert world.attrib["name"] == "warehouse_mission_world"
    model_names = {model.attrib["name"] for model in world.findall("model")}
    assert {"warehouse_amr", "warehouse_structure", "movable_test_cart"} <= model_names

    metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    assert metadata["image"] == "warehouse_map.pgm"
    assert metadata["resolution"] == 0.05
    width, height, _pixels = read_pgm(map_image_path)
    assert (width, height) == (560, 480)


def test_every_mission_pose_is_free_in_static_map() -> None:
    metadata = yaml.safe_load(
        (PROJECT_ROOT / "maps" / "warehouse_map.yaml").read_text(encoding="utf-8")
    )
    mission = yaml.safe_load(
        (
            PROJECT_ROOT
            / "src"
            / "amr_mission_manager"
            / "config"
            / "warehouse_demo.yaml"
        ).read_text(encoding="utf-8")
    )["mission"]
    width, height, pixels = read_pgm(
        PROJECT_ROOT / "maps" / "warehouse_map.pgm"
    )
    origin_x, origin_y, _yaw = metadata["origin"]
    resolution = metadata["resolution"]

    poses = [mission["home"], *mission["tasks"]]
    for pose in poses:
        column = int((pose["x"] - origin_x) / resolution)
        row_from_bottom = int((pose["y"] - origin_y) / resolution)
        row = height - 1 - row_from_bottom
        assert 0 <= column < width and 0 <= row < height
        assert pixels[row * width + column] >= 250, pose.get("name", pose.get("id"))


def test_runtime_files_do_not_reference_project_01() -> None:
    runtime_roots = [
        PROJECT_ROOT / "config",
        PROJECT_ROOT / "maps",
        PROJECT_ROOT / "rviz",
        PROJECT_ROOT / "scripts",
        PROJECT_ROOT / "src",
        PROJECT_ROOT / "worlds",
        PROJECT_ROOT / "run_demo.sh",
    ]
    forbidden = ("/home/ziling/projects/amr_slam", "~/projects/amr_slam")
    for root in runtime_roots:
        paths = [root] if root.is_file() else root.rglob("*")
        for path in paths:
            if (
                path.is_file()
                and path.suffix not in {".pgm", ".pyc"}
                and "__pycache__" not in path.parts
            ):
                text = path.read_text(encoding="utf-8")
                assert not any(value in text for value in forbidden), path
