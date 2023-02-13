mod shell_approach_static;

use std::fs::File;
use std::io::BufReader;
use itertools::Itertools;
use obj::{load_obj, Obj};
use parry3d::na::Point3;
use parry3d::shape::TriMesh;

use bevy::prelude::*;

fn main() {



    App::new()
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup)
        .run();


}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {

    let urdf_file = File::open("/home/werner/dev_ws/src/motion-planning-around-apple-trees/3d-models/appletree_trunk.obj").unwrap();
    let trunk_model: Obj = load_obj(BufReader::new(urdf_file)).expect("Failed to load obj file.");

    let trunk_shape = TriMesh::new(
        trunk_model.vertices.iter().map(|v| Point3::new(v.position[0], v.position[1], v.position[2])).collect(),
        trunk_model.indices
            .iter()
            .chunks(3)
            .into_iter()
            .map(|chunk| {
            let mut chunk = chunk.into_iter();
            let a = chunk.next().unwrap();
            let b = chunk.next().unwrap();
            let c = chunk.next().unwrap();
            [*a as u32, *b as u32, *c as u32]
        }).collect(),
    );

    let urdf_robot = urdf_rs::read_file("/home/werner/dev_ws/src/motion-planning-around-apple-trees/test_robots/urdf/bot.urdf").unwrap();

    for link in urdf_robot.links {
        println!("Link: {}", link.name);
    }

    // plane
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane { size: 5.0 })),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });
    // cube
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..default()
    });
    // light
    commands.spawn_bundle(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    // camera
    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
