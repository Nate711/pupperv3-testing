import xml.etree.ElementTree as ET
import os
import pathlib
import numpy as np
import yaml


def header(config):
    compiler = ET.Element("compiler",
                          angle="radian",
                          meshdir=config["mesh_dir"])
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
    ET.SubElement(asset,
                  "texture",
                  type="skybox",
                  builtin="gradient",
                  rgb1=".3 .5 .7",
                  rgb2="0 0 0",
                  width="512",
                  height="512")
    ET.SubElement(asset,
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
                  random="0.01")
    ET.SubElement(asset,
                  "material",
                  name="body",
                  texture="body",
                  texuniform="true",
                  rgba="0.8 0.6 .4 1")
    ET.SubElement(asset,
                  "texture",
                  name="grid",
                  type="2d",
                  builtin="checker",
                  width="512",
                  height="512",
                  rgb1=".1 .2 .3",
                  rgb2=".2 .3 .4")
    ET.SubElement(asset,
                  "material",
                  name="grid",
                  texture="grid",
                  texrepeat="1 1",
                  texuniform="true",
                  reflectance=".2")
    return asset


def defaults(config):
    default = ET.Element("default")
    ET.SubElement(default,
                  "motor",
                  forcelimited="true",
                  forcerange=vec2str(config["actuator"]["forcerange"]),
                  ctrlrange=vec2str(config["actuator"]["ctrlrange"]),
                  ctrllimited="true")
    body_default = ET.SubElement(default, "default", {"class": "body"})
    ET.SubElement(body_default,
                  "geom",
                  condim="1",
                  friction=str(config["friction"]),
                  solimp=".9 .99 .003",
                  solref=".015 1",
                  material="body")
    # TODO: Specify gim4305 actuator class
    ET.SubElement(body_default,
                  "joint",
                  type="hinge",
                  damping=str(config["actuator"]["damping"]),
                  frictionloss=str(config["actuator"]["frictionloss"]),
                  limited="true",
                  solimplimit="0 0.99 0.01")
    return default


def world_body():
    world_body = ET.Element("world_body")
    ET.SubElement(world_body,
                  "geom",
                  name="floor",
                  size="0 0 .05",
                  type="plane",
                  material="grid",
                  condim="3")
    ET.SubElement(world_body,
                  "light",
                  name="spotlight",
                  mode="targetbodycom",
                  target="torso",
                  diffuse=".8 .8 .8",
                  specular="0.3 0.3 0.3",
                  pos="0 -4 4",
                  cutoff="100")
    return world_body


def torso(config):
    torso = ET.Element("body",
                       name="torso",
                       pos=vec2str(config["torso_pos"]),
                       childclass="body")
    if config["floating_base"]:
        ET.SubElement(torso, "freejoint", name="root")
        ET.SubElement(torso,
                      "inertial",
                      pos="0 0 0",
                      mass="1.6",
                      diaginertia="2e-03 2e-03 2e-03")
    ET.SubElement(torso,
                  "geom",
                  pos="-0.0019 -0.00027 0.02901",
                  quat="0.707105 0.707108 0 0",
                  type="mesh",
                  contype="0",
                  conaffinity="0",
                  group="1",
                  density="0",
                  mesh="ProtoAssembly-MGv25.004")
    ET.SubElement(torso,
                  "geom",
                  size="0.12009 0.05097 0.0575",
                  pos="9e-05 0 0.02177",
                  quat="0.707105 0.707108 0 0",
                  type="box")
    return torso


def lower_leg(side, location, config):
    name = f"{location}_{side}_lower"
    body = ET.Element("body",
                      name=name,
                      pos="-0.0607 0 0.07062",
                      quat="0.21263 0.212631 0.674379 -0.674381")
    ET.SubElement(body,
                  "inertial",
                  pos="0.08114 -0.0073 0.01746",
                  mass=config["lower_leg_mass"],
                  diaginertia="2e-05 2e-05 2e-05")
    ET.SubElement(body,
                  "joint",
                  armature=str(config["actuator"]["armature"]),
                  name=name,
                  pos="0 0 0",
                  axis="0 0 1",
                  limited="true",
                  range="-3.14159 1.20428")
    ET.SubElement(
        body,
        "geom",
        pos="0.08114 -0.0073 0.01746",
        type="mesh",
        contype="0",
        conaffinity="0",
        group="1",
        density="0",
        mesh=config["lower_link_mesh"],
    )
    ET.SubElement(body, "geom", size="0.01984", pos="0.09946 -0.0107 0.0169")
    return body


def vec2str(vec):
    res = ""
    for e in vec:
        res += str(e) + " "
    return res


def flip_quat(quat, flip):
    quat_ = np.array(quat)
    return quat_ * [-1, 1, 1, 1] if flip else quat_


def flip_tuple(tuple, flip):
    if flip:
        return np.array(tuple[::-1])
    else:
        return np.array(tuple)


def mirror_y(pos_vec, mirror):
    if mirror:
        return np.array(pos_vec) * [1, -1, 1]
    else:
        return np.array(pos_vec)


def hip(side, location, config):
    name = f"{location}_{side}_hip"
    hip_location = config["hip"]["location"][location][side]
    hip_quat = flip_quat(config["hip"]["right_orientation"],
                         flip=side == "left")
    body = ET.Element("body",
                      name=name,
                      pos=hip_location,
                      quat=vec2str(hip_quat))
    hip_com = mirror_y(config["hip"]["right_com"], mirror=side == "left")
    ET.SubElement(body,
                  "inertial",
                  pos=vec2str(hip_com),
                  mass=str(config["hip"]["mass"]),
                  diaginertia="2e-05 2e-05 2e-05")
    joint_limits = flip_tuple(config["hip"]["right_limits"],
                              flip=side == "left")
    ET.SubElement(body,
                  "joint",
                  armature=str(config["actuator"]["armature"]),
                  name=name,
                  pos="0 0 0",
                  axis="0 0 1",
                  limited="true",
                  range=vec2str(joint_limits))
    hip_mesh_pos = mirror_y(config["hip"]["right_mesh_pos"],
                            mirror=side == "left")
    ET.SubElement(
        body,
        "geom",
        pos=vec2str(hip_mesh_pos),
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
    body = ET.Element("body",
                      name=name,
                      pos="0 0 0.038",
                      quat="0.499998 -0.5 -0.500002 0.5")
    ET.SubElement(body,
                  "inertial",
                  pos="-0.04555 -0.00233 0.05745",
                  quat="0.5 0.5 -0.5 0.5",
                  mass=config["upper_leg_mass"],
                  diaginertia="2e-05 2e-05 2e-05")
    ET.SubElement(body,
                  "joint",
                  armature=str(config["actuator"]["armature"]),
                  name=name,
                  pos="0 0 0",
                  axis="0 0 1",
                  limited="true",
                  range="-0.69813 1.5708")
    ET.SubElement(
        body,
        "geom",
        pos="-0.04555 -0.00233 0.05745",
        quat="0.499998 0.5 0.5 -0.500002",
        type="mesh",
        contype="0",
        conaffinity="0",
        group="1",
        density="0",
        mesh=config["upper_leg_mesh"],
    )
    ET.SubElement(body, "geom", size="0.025", pos="-0.0607 -0.005 0.07062")
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
    for (side, location) in [("right", "front"), ("left", "front"),
                             ("right", "back"), ("left", "back")]:
        base_name = f"{location}_{side}"
        ET.SubElement(actuator,
                      "motor",
                      gear=str(config["actuator"]["gear"]),
                      joint=f"{base_name}_hip",
                      name=f"{base_name}_hip")
        ET.SubElement(actuator,
                      "motor",
                      gear=str(config["actuator"]["gear"]),
                      joint=f"{base_name}_upper",
                      name=f"{base_name}_upper")
        ET.SubElement(actuator,
                      "motor",
                      gear=str(config["actuator"]["gear"]),
                      joint=f"{base_name}_lower",
                      name=f"{base_name}_lower")
    return actuator


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
    "torso_pos": [0, 0, 0.2],
    "hip": {
        "mass": 0.16,
        "mesh": {
            "right": "ProtoAssembly-MGv25.007",
            "left": "ProtoAssembly-MGv25.003"
        },
        "location": {
            "front": {
                "right": "0.085 -0.05 0",
                "left": "0.085 0.05 0",
            },
            "back": {
                "right": "-0.085 -0.05 0",
                "left": "-0.085 0.05 0",
            }
        },
        "right_orientation": [
            0.707106,
            0.707106,
            0,
            0,
        ],
        "right_limits": [-2.35619, 2.87979],
        "right_mesh_pos": [0.00272, -1e-05, 0.03409],
        "right_com": [0.00272, -1e-05, 0.03409],
    },
    "upper_leg_mesh": "ProtoAssembly-MGv25.005",
    "upper_leg_mass": "0.17",
    "lower_link_mesh": "ProtoAssembly-MGv25.006",
    "lower_leg_mass": "0.045",
}

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