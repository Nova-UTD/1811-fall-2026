# Load the shipped vehicle profile through the registration package's validator.

from pathlib import Path
from types import SimpleNamespace

from lidar_camera_calibration.config import load_registration_config
import yaml

SOURCE = Path(__file__).resolve().parents[1]
PROFILE = SOURCE / 'config' / 'registration.yaml'


def vehicle_config(overrides=None):
    values = yaml.safe_load(PROFILE.read_text())['lidar_camera_registration']['ros__parameters']
    for name, camera in values.pop('cameras').items():
        for parameter, value in camera.items():
            values[f'cameras.{name}.{parameter}'] = value
    values.update(overrides or {})
    node = SimpleNamespace(declare_parameter=lambda name, default, _descriptor: SimpleNamespace(
        value=values.get(name, default),
    ))
    return load_registration_config(node)


def vehicle_description(config):
    links = [f'<link name="{config.reference_frame}"/>']
    joints = []
    for camera in config.cameras.values():
        links.append(f'<link name="{camera.body_frame}"/>')
        joints.append(
            f'<joint name="{camera.mount_joint}" type="floating">'
            f'<parent link="{camera.parent_frame}"/>'
            f'<child link="{camera.body_frame}"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>'
        )
    return f'<robot name="{config.robot_name}">{"".join(links + joints)}</robot>'
