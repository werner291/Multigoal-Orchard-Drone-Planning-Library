mod shell_approach_static;
mod robot;
mod drone_with_arm;

use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use itertools::Itertools;
use obj::{load_obj, Obj};
use parry3d::na::Point3;
use parry3d::shape::{Shape, TriMesh};

use bevy::prelude::*;

fn main() {

    App::new()
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup)
        .run();

}

struct CycleInRobotError;

impl TryFrom<urdf_rs::Robot> for HierarchicalRigidRobot {

    type Error = CycleInRobotError;

    fn try_from(value: Robot) -> Result<Self, Self::Error> {

        // Put all the links into a map by name.
        let links = value.links.into_iter().map(|link| (link.name.clone(), link)).collect::<HashMap<_, _>>();



    }
}


/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {

    let urdf_file = File::open("/home/werner/dev_ws/src/motion-planning-around-apple-trees/3d-models/appletree_trunk.obj").unwrap();
    let trunk_model: Obj = load_obj(BufReader::new(urdf_file)).expect("Failed to load obj file.");
    let trunk_shape = obj_to_parry_trimesh(trunk_model);

    let urdf_robot = urdf_rs::read_file("/home/werner/dev_ws/src/motion-planning-around-apple-trees/test_robots/urdf/bot.urdf").unwrap();

    for link in urdf_robot.links {
        for col in &link.collision {
            let mesh = urdf_shape_to_bevy_mesh(&col.geometry);
            commands.spawn_bundle(PbrBundle {
                mesh: meshes.add(mesh),
                material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                transform: Transform::from_xyz(0.0, 0.5, 0.0),
                ..default()
            });
        }
    }

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

fn obj_to_parry_trimesh(trunk_model: Obj) -> TriMesh {
    TriMesh::new(
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
    )
}

fn urdf_shape_to_bevy_mesh(geometry: &Geometry) -> Mesh {
    let mesh = match geometry {
        Geometry::Box { size } => Mesh::from(shape::Box {
            min_x: (-size[0] / 2.0) as f32,
            min_y: (-size[1] / 2.0) as f32,
            min_z: (-size[2] / 2.0) as f32,
            max_x: (size[0] / 2.0) as f32,
            max_y: (size[1] / 2.0) as f32,
            max_z: (size[2] / 2.0) as f32,
        }),
        Geometry::Cylinder { .. } => todo!(),
        Geometry::Capsule { .. } => todo!(),
        Geometry::Sphere { .. } => todo!(),
        Geometry::Mesh { .. } => todo!(),
    };
    mesh
}
