#!/usr/bin/env python3
"""Regenerate the checked-in real geometry and SDF worlds from source."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from terrain_geometry_impl.demo_scene import ramp_mesh, rough_mesh  # noqa: E402
from terrain_geometry_impl.l1_profile import L1Profile  # noqa: E402


def write_stl(path, vertices, faces):
    with path.open('w') as stream:
        stream.write('solid terrain\n')
        for face in faces:
            triangle = vertices[face]
            normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
            normal /= np.linalg.norm(normal)
            stream.write('facet normal ' + ' '.join(f'{v:.9g}' for v in normal) + '\nouter loop\n')
            for vertex in triangle:
                stream.write('vertex ' + ' '.join(f'{v:.9g}' for v in vertex) + '\n')
            stream.write('endloop\nendfacet\n')
        stream.write('endsolid terrain\n')


def add(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def object_model(world, name, shape, value, pose, color):
    model = add(world, 'model', name=name)
    add(model, 'static', 'true')
    add(model, 'pose', pose)
    link = add(model, 'link', name='link')
    for kind in ('collision', 'visual'):
        part = add(link, kind, name=kind)
        geometry = add(part, 'geometry')
        primitive = add(geometry, shape)
        add(primitive, 'uri' if shape == 'mesh' else 'size', value)
        if kind == 'visual':
            material = add(part, 'material')
            add(material, 'ambient', color)
            add(material, 'diffuse', color)


def make_world(stand, variant='PM'):
    profile = L1Profile(variant=variant)
    sdf = ET.Element('sdf', version='1.9')
    world = add(sdf, 'world', name='terrain_demo' if stand else 'terrain_rover')
    physics = add(world, 'physics', name='physics', type='ignored')
    add(physics, 'max_step_size', '0.001')
    add(physics, 'real_time_factor', '1.0')
    for short, system in (('physics', 'Physics'), ('user-commands', 'UserCommands'),
                          ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'),
                          ('imu', 'Imu')):
        plugin = add(world, 'plugin', filename=f'gz-sim-{short}-system', name=f'gz::sim::systems::{system}')
        if short == 'sensors':
            add(plugin, 'render_engine', 'ogre2')
    add(world, 'gravity', '0 0 -9.81')
    light = add(world, 'light', name='sun', type='directional')
    add(light, 'pose', '0 0 10 0 0 0')
    add(light, 'diffuse', '0.8 0.8 0.8 1')
    add(light, 'direction', '-0.5 0.1 -0.9')
    object_model(world, 'flat_ground', 'box', '30 30 0.2', '0 0 -0.1 0 0 0', '0.55 0.50 0.40 1')
    object_model(world, 'ramp_15_degrees', 'mesh', 'model://terrain_geometry/meshes/ramp.stl',
                 '0 0 0 0 0 0', '0.20 0.55 0.75 1')
    object_model(world, 'uneven_ground', 'mesh', 'model://terrain_geometry/meshes/rough.stl',
                 '0 0 0 0 0 0', '0.55 0.35 0.20 1')
    object_model(world, 'raised_obstacle_0_7m', 'box', '0.9 1.0 0.7', '3.5 0 0.35 0 0 0', '0.8 0.2 0.15 1')
    if stand:
        model = add(world, 'model', name='stationary_unitree_l1_' + variant.lower())
        add(model, 'static', 'true')
        add(model, 'pose', f'0 0 1.2 {profile.mount_roll} 0 0')
        link = add(model, 'link', name=profile.lidar_frame)
        # Virtual ray origin at the SDK frame origin; no CAD housing/self-return model.
        sensor = add(link, 'sensor', name='unitree_l1_approximation', type='gpu_lidar')
        add(sensor, 'topic', '/unilidar')
        add(sensor, 'gz_frame_id', profile.lidar_frame)
        add(sensor, 'always_on', 'true')
        add(sensor, 'update_rate', profile.update_rate)
        add(sensor, 'visualize', 'false')
        lidar = add(sensor, 'lidar')
        scan = add(lidar, 'scan')
        for name, samples, lo, hi in (('horizontal', profile.horizontal_samples, -np.pi, np.pi),
                                      ('vertical', profile.vertical_samples, 0., np.pi/2)):
            axis = add(scan, name)
            for tag, value in (('samples', samples), ('resolution', 1), ('min_angle', lo), ('max_angle', hi)):
                add(axis, tag, value)
        distance = add(lidar, 'range')
        for tag, value in (('min', profile.min_range), ('max', profile.max_range),
                           ('resolution', profile.range_resolution)):
            add(distance, tag, value)
        noise = add(lidar, 'noise', type='gaussian')
        add(noise, 'mean', 0.0)
        add(noise, 'stddev', profile.range_noise_stddev)
        imu = add(link, 'sensor', name='unitree_l1_imu_approximation', type='imu')
        add(imu, 'pose', ' '.join(map(str, profile.imu_offset)) + ' 0 0 0')
        add(imu, 'topic', '/unilidar/imu')
        add(imu, 'gz_frame_id', profile.imu_frame)
        add(imu, 'always_on', 'true')
        add(imu, 'update_rate', profile.imu_rate)
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='unicode', xml_declaration=True)


if __name__ == '__main__':
    (ROOT/'meshes').mkdir(exist_ok=True)
    (ROOT/'worlds').mkdir(exist_ok=True)
    for name, factory in (('ramp', ramp_mesh), ('rough', rough_mesh)):
        write_stl(ROOT/'meshes'/f'{name}.stl', *factory())
    for stand, name in ((True, 'terrain_demo'), (False, 'terrain_rover')):
        (ROOT/'worlds'/f'{name}.sdf').write_text(make_world(stand) + '\n')
    (ROOT/'worlds'/'terrain_demo_l1_rm.sdf').write_text(make_world(True, 'RM') + '\n')
