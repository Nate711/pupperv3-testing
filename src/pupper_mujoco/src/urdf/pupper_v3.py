import xml.etree.ElementTree as ET
import os
import pathlib
import numpy as np
import yaml
from geometry_utils import vec2str
import geometry_utils


def header(config):
    compiler = ET.Element("compiler", angle="radian", meshdir=config["mesh_dir"])
    size = ET.Element("size", njmax="500", nconmax="100")
    return [compiler, size]


def mesh_assets(config):
    asset = ET.Element("asset")
    mesh_files = os.listdir(config["mesh_dir"])
    for mesh_file in mesh_files:
        base_name = pathlib.Path(mesh_file).stem
        ET.SubElement(asset, "mesh", name=base_name, file=mesh_file)
    return asset


def ground_plane_assets():
    asset = ET.Element("asset")
    ET.SubElement(
        asset,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1=".3 .5 .7",
        rgb2="0 0 0",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "texture",
        name="body",
        type="cube",
        builtin="flat",
        mark="cross",
        width="127",
        height="1278",
        rgb1="0.8 0.6 0.4",
        rgb2="0.8 0.6 0.4",
        markrgb="1 1 1",
        random="0.01",
    )
    ET.SubElement(
        asset,
        "material",
        name="body",
        texture="body",
        texuniform="true",
        rgba="0.8 0.6 .4 1",
    )
    ET.SubElement(
        asset,
        "texture",
        name="grid",
        type="2d",
        builtin="checker",
        width="512",
        height="512",
        rgb1=".1 .2 .3",
        rgb2=".2 .3 .4",
    )
    ET.SubElement(
        asset,
        "material",
        name="grid",
        texture="grid",
        texrepeat="1 1",
        texuniform="true",
        reflectance=".2",
    )
    return asset


def defaults(config):
    default = ET.Element("default")
    ET.SubElement(
        default,
        "motor",
        forcelimited="true",
        forcerange=vec2str(config["actuator"]["forcerange"]),
        ctrlrange=vec2str(config["actuator"]["ctrlrange"]),
        ctrllimited="true",
    )
    body_default = ET.SubElement(default, "default", {"class": "body"})
    ET.SubElement(
        body_default,
        "geom",
        condim="1",
        friction=str(config["friction"]),
        solimp=".9 .99 .003",
        solref=".015 1",
        material="body",
    )
    # TODO: Specify gim4305 actuator class
    ET.SubElement(
        body_default,
        "joint",
        type="hinge",
        damping=str(config["actuator"]["damping"]),
        frictionloss=str(config["actuator"]["frictionloss"]),
        limited="true",
        solimplimit="0 0.99 0.01",
    )
    return default


def world_body():
    world_body = ET.Element("world_body")
    ET.SubElement(
        world_body,
        "geom",
        name="floor",
        size="0 0 .05",
        type="plane",
        material="grid",
        condim="3",
    )
    ET.SubElement(
        world_body,
        "light",
        name="spotlight",
        mode="targetbodycom",
        target="torso",
        diffuse=".8 .8 .8",
        specular="0.3 0.3 0.3",
        pos="0 -4 4",
        cutoff="100",
    )
    return world_body


def torso(config):
    torso = ET.Element(
        "body", name="torso", pos=vec2str(config["torso"]["pos"]), childclass="body"
    )
    if config["floating_base"]:
        ET.SubElement(torso, "freejoint", name="root")
        ET.SubElement(
            torso,
            "inertial",
            pos=vec2str(config["torso"]["com"]),
            mass=str(config["torso"]["mass"]),
            diaginertia=vec2str(config["torso"]["fullinertia"]),
        )
    ET.SubElement(
        torso,
        "geom",
        pos=vec2str(config["torso"]["mesh_pos"]),
        quat=vec2str(config["torso"]["mesh_orientation"]),
        type="mesh",
        contype="0",
        conaffinity="0",
        group="1",
        density="0",
        mesh=config["torso"]["mesh"],
    )
    ET.SubElement(
        torso,
        "geom",
        size=vec2str(config["torso"]["collision_size"]),
        pos=vec2str(config["torso"]["collision_pos"]),
        quat=vec2str(config["torso"]["collision_quat"]),
        type="box",
    )
    return torso


def lower_leg(side, location, config):
    name = f"{location}_{side}_lower"
    body = ET.Element(
        "body",
        name=name,
        pos=vec2str(config["lower"]["pos"][side]),
        quat=vec2str(config["lower"]["orientation"][side]),
    )
    ET.SubElement(
        body,
        "inertial",
        pos=vec2str(config["lower"]["com"][side]),
        mass=str(config["lower"]["mass"]),
        diaginertia=vec2str(config["lower"]["fullinertia"][side]),
    )
    ET.SubElement(
        body,
        "joint",
        armature=str(config["actuator"]["armature"]),
        name=name,
        pos="0 0 0",
        axis="0 0 1",
        limited="true",
        range=vec2str(config["lower"]["limits"][side]),
    )
    ET.SubElement(
        body,
        "geom",
        pos=vec2str(config["lower"]["mesh_pos"][side]),
        type="mesh",
        contype="0",
        conaffinity="0",
        group="1",
        density="0",
        mesh=config["lower"]["mesh"][side],
    )
    ET.SubElement(
        body,
        "geom",
        size=config["lower"]["ball_size"],
        pos=vec2str(config["lower"]["ball_pos"]),
    ),
    return body


def hip(side, location, config):
    name = f"{location}_{side}_hip"
    body = ET.Element(
        "body",
        name=name,
        pos=vec2str(config["hip"]["location"][location][side]),
        quat=vec2str(config["hip"]["orientation"][side]),
    )
    ET.SubElement(
        body,
        "inertial",
        pos=vec2str(config["hip"]["com"][side]),
        mass=str(config["hip"]["mass"]),
        fullinertia=vec2str(config["hip"]["fullinertia"][side]),
    )
    ET.SubElement(
        body,
        "joint",
        armature=str(config["actuator"]["armature"]),
        name=name,
        pos="0 0 0",
        axis="0 0 1",
        limited="true",
        range=vec2str(config["hip"]["limits"][side]),
    )
    ET.SubElement(
        body,
        "geom",
        pos=vec2str(config["hip"]["mesh_pos"][side]),
        type="mesh",
        contype="0",
        conaffinity="0",
        group="1",
        density="0",
        mesh=config["hip"]["mesh"][side],
    )
    return body


def upper_leg(side, location, config):
    name = f"{location}_{side}_upper"
    body = ET.Element(
        "body",
        name=name,
        pos=vec2str(config["upper"]["pos"][side]),
        quat=vec2str(config["upper"]["orientation"][side]),
    )
    ET.SubElement(
        body,
        "inertial",
        pos=vec2str(config["upper"]["com"][side]),
        quat=vec2str(config["upper"]["inertia_orientation"][side]),
        mass=str(config["upper"]["mass"]),
        diaginertia=vec2str(config["upper"]["fullinertia"][side]),
    )
    ET.SubElement(
        body,
        "joint",
        armature=str(config["actuator"]["armature"]),
        name=name,
        pos="0 0 0",
        axis="0 0 1",
        limited="true",
        range=vec2str(config["upper"]["limits"][side]),
    )
    ET.SubElement(
        body,
        "geom",
        pos=vec2str(config["upper"]["mesh_pos"][side]),
        quat=vec2str(config["upper"]["mesh_orientation"][side]),
        type="mesh",
        contype="0",
        conaffinity="0",
        group="1",
        density="0",
        mesh=config["upper"]["mesh"][side],
    )
    ET.SubElement(
        body,
        "geom",
        size=str(config["upper"]["ball_size"]),
        pos=vec2str(config["upper"]["ball_pos"][side]),
    )
    return body


def full_leg(side, location, config):
    lower_ = lower_leg(side, location, config)
    upper_ = upper_leg(side, location, config)
    hip_ = hip(side, location, config)
    upper_.append(lower_)
    hip_.append(upper_)
    return hip_


def actuators(config):
    actuator = ET.Element("actuator")
    # TODO: encode that the order is important
    for (side, location) in [
        ("right", "front"),
        ("left", "front"),
        ("right", "back"),
        ("left", "back"),
    ]:
        base_name = f"{location}_{side}"
        ET.SubElement(
            actuator,
            "motor",
            gear=str(config["actuator"]["gear"]),
            joint=f"{base_name}_hip",
            name=f"{base_name}_hip",
        )
        ET.SubElement(
            actuator,
            "motor",
            gear=str(config["actuator"]["gear"]),
            joint=f"{base_name}_upper",
            name=f"{base_name}_upper",
        )
        ET.SubElement(
            actuator,
            "motor",
            gear=str(config["actuator"]["gear"]),
            joint=f"{base_name}_lower",
            name=f"{base_name}_lower",
        )
    return actuator


def construct_left_side(config):
    def mirror_element(e):
        if len(e) == 2:
            return geometry_utils.flip_tuple(e)
        if len(e) == 3:
            return geometry_utils.mirror_y(e)
        if len(e) == 4:
            return geometry_utils.flip_quat(e)
        if len(e) == 6:
            return geometry_utils.mirror_inertia_y(e)

    def recursive_fn(elem):
        if isinstance(elem, dict):
            if "right" in elem and "left" not in elem:
                elem["left"] = mirror_element(elem["right"])
            else:
                for key in elem:
                    recursive_fn(elem[key])
        else:
            pass

    recursive_fn(config["hip"])
    recursive_fn(config["upper"])
    recursive_fn(config["lower"])


if __name__ == "__main__":
    config = {
        "floating_base": True,
        "friction": 1.0,
        "mesh_dir": "../meshes/",
        "actuator": {
            "gear": 1.0,
            "armature": 0.0016,
            "damping": 0.001,
            "frictionloss": 0.02,
            "forcerange": [-3, 3],
            "ctrlrange": [-3, 3],
        },
        "torso": {
            "pos": [0, 0, 0.2],
            "com": [0, 0, 0],
            "mass": 1.6,
            "fullinertia": [2e-03, 2e-03, 2e-03, 0, 0, 0],
            "mesh_pos": [-0.0019, -0.00027, 0.02901],
            "mesh_orientation": [0.707105, 0.707108, 0, 0],
            "mesh": "ProtoAssembly-MGv25.004",
            "collision_size": [0.12009, 0.05097, 0.0575],
            "collision_pos": [9e-05, 0, 0.02177],
            "collision_quat": [0.707105, 0.707108, 0, 0],
        },
        "hip": {
            "location": {
                "front": {"right": [0.085, -0.05, 0]},
                "back": {"right": [-0.085, -0.05, 0]},
            },
            "mass": 0.16,
            "fullinertia": {"right": [2e-5, 2e-5, 2e-5, 0, 0, 0]},
            "orientation": {
                "right": [
                    0.707106,
                    0.707106,
                    0,
                    0,
                ]
            },
            "limits": {"right": [-2.35619, 2.87979]},
            "mesh_pos": {"right": [0.00272, -1e-05, 0.03409]},
            "com": {"right": [0.00272, -1e-05, 0.03409]},
            "mesh": {
                "right": "ProtoAssembly-MGv25.007",
                "left": "ProtoAssembly-MGv25.003",
            },
        },
        "upper": {
            "pos": {"right": [0, 0, 0.038]},
            "orientation": {"right": [0.499998, -0.5, -0.500002, 0.5]},
            "com": {"right": [-0.04555, -0.00233, 0.05745]},
            "mass": 0.17,
            "inertia_orientation": {"right": [0.5, 0.5, -0.5, 0.5]},
            "fullinertia": {"right": [2e-05, 2e-05, 2e-05, 0, 0, 0]},
            "limits": {"right": [-0.69813, 1.5708]},
            "mesh_pos": {"right": [-0.04555, -0.00233, 0.05745]},
            "mesh_orientation": {"right": [0.499998, 0.5, 0.5, -0.500002]},
            "mesh": {
                "right": "ProtoAssembly-MGv25.010",
                "left": "ProtoAssembly-MGv25.014",
            },
            "ball_pos": {"right": [-0.0607, -0.005, 0.07062]},
            "ball_size": 0.025,
        },
        "lower": {
            "mass": 0.045,
            "pos": {"right": [-0.0607, 0, 0.07062]},
            "orientation": {"right": [0.21263, 0.212631, 0.674379, -0.674381]},
            "com": {"right": [0.08114, -0.0073, 0.01746]},
            "fullinertia": {"right": [2e-05, 2e-05, 2e-05, 0, 0, 0]},
            "limits": {
                "right": [-3.14159, 1.20428],
            },
            "mesh_pos": {
                "right": [0.08114, -0.0073, 0.01746],
            },
            "mesh": {
                "right": "ProtoAssembly-MGv25.006",
                "left": "ProtoAssembly-MGv25.002",
            },
            "ball_pos": {
                "right": [0.09946, -0.0107, 0.0169],
            },
            "ball_size": str(0.01984),
        },
    }

    construct_left_side(config)

    mujoco_model = ET.Element("mujoco", model="pupper_v3")
    mujoco_model.extend(header(config))
    mujoco_model.append(mesh_assets(config))
    mujoco_model.append(ground_plane_assets())
    mujoco_model.append(defaults(config))

    torso_ = torso(config)
    torso_.append(full_leg("right", "front", config))
    torso_.append(full_leg("left", "front", config))
    torso_.append(full_leg("right", "back", config))
    torso_.append(full_leg("left", "back", config))

    world_body_ = world_body()
    world_body_.append(torso_)
    mujoco_model.append(world_body_)
    mujoco_model.append(actuators(config))

    tree = ET.ElementTree(mujoco_model)
    ET.indent(tree, "\t")
    tree.write("pupper_v3_et_gen.xml")

    with open("pupper_v3_config.yaml", "w") as f:
        f.write(yaml.dump(config))
