#![feature(test)]
extern crate test;

pub(crate) mod bhtree;
pub(crate) mod bodies;
pub(crate) mod tests;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use bhtree::{Quad, Quadtree};
use bodies::Body;
use rand::Rng;
use std::ops::RangeInclusive;

mod collision;
use collision::collision;

#[derive(Resource)]
pub struct SimulationSettings {
    // live tweakables
    delta_t: f32,
    g: f32,
    show_tree: bool,
    // needs simulation reset
    min_body_mass: f32,
    max_body_mass: f32,
    n_bodies: u32,
    z: f32,
    spawn_area: RangeInclusive<f32>,
    theta: f32,
    init_vel: f32,
    donut: bool,
    elasticity: f32,
    collision_enabled: bool,
}

impl Default for SimulationSettings {
    fn default() -> Self {
        SimulationSettings {
            delta_t: 0.001,
            g: 1.0,
            show_tree: false,
            min_body_mass: 10.0,
            max_body_mass: 100.0,
            n_bodies: 1500,
            z: 10.0,
            spawn_area: -300.0..=300.0,
            theta: 0.5,
            init_vel: 50.0,
            donut: false,
            elasticity: 1.0,
            collision_enabled: false,
        }
    }
}

#[derive(Component)]
pub struct Velocity(Vec2);

#[derive(Resource)]
struct QuadtreeResource {
    tree: Quadtree,
}

#[derive(Message)]
struct ResetMessage;

pub fn main() {
    let app = App::new();
    setup_app(app).run();
}

fn setup_app(mut app: App) -> App {
    app.insert_resource(ClearColor(Color::BLACK))
        .insert_resource(SimulationSettings::default())
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin::default())
        .add_message::<ResetMessage>()
        .add_systems(EguiPrimaryContextPass, ui_window)
        .add_systems(
            Startup,
            (spawn_camera, add_bodies),
        )
        .add_systems(
            Update,
            (
                collision,
                reset_handler,
                draw_bodies,
                (
                    build_quadtree,
                    compute_physics,
                    update_positions,
                    draw_tree,
                ).chain()
            ),
        );

    app
}

fn ui_window(
    mut contexts: EguiContexts,
    mut settings: ResMut<SimulationSettings>,
    mut reset_writer: MessageWriter<ResetMessage>,
) -> Result {
    egui::Window::new("Settings").show(contexts.ctx_mut().unwrap(), |ui| {
        ui.add(egui::Slider::new(&mut settings.g, 0.0..=10.0).text("Gravity constant"));
        ui.add(egui::Slider::new(&mut settings.delta_t, 0.00000001..=0.01).text("Delta T"));
        ui.add(egui::Slider::new(&mut settings.theta, 0.1..=1.0).text("BH Theta"));
        ui.add(egui::Checkbox::new(
            &mut settings.show_tree,
            "Draw Quadtree",
        ));
        ui.add(egui::Checkbox::new(
            &mut settings.collision_enabled,
            "Enable Collision",
        ));
        ui.add(egui::Slider::new(&mut settings.elasticity, 0.0..=1.0).text("Elasticity"));

        ui.add(egui::Label::new("Reset Sim after tweaking these:"));
        ui.add(egui::Slider::new(&mut settings.n_bodies, 2..=50000).text("Num Bodies"));
        ui.add(egui::Slider::new(&mut settings.min_body_mass, 1.0..=5000.0).text("Min Body Mass"));
        ui.add(egui::Slider::new(&mut settings.max_body_mass, 1.0..=5000.0).text("Max Body Mass"));
        ui.add(egui::Checkbox::new(&mut settings.donut, "Donut Start"));
        ui.add(
            egui::Slider::new(&mut settings.init_vel, 1.0..=1000.0)
                .text("Initial Velocity (Only Donut)"),
        );
        if ui.button("Reset").clicked() {
            reset_writer.write(ResetMessage);
        }
    });
    Ok(())
}

fn reset_handler(
    query: Query<Entity, With<Body>>,
    reset_event: MessageReader<ResetMessage>,
    mut commands: Commands,
    settings: Res<SimulationSettings>,
) {
    if reset_event.is_empty() {
        return;
    }

    for entity in &query {
        commands.entity(entity).despawn();
    }

    // add_bodies(commands, materials, meshes, settings);
    spawn_bodies_from_settings(commands, &settings);
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn(Camera2d::default());
}

fn mass_to_radius(m: f32) -> f32 {
    // Radius determination: r=sqrt(G * Mass/accel)
    let g = 1.0; // constant for size
    let a = 10.0; // initial accel
    let res = g * m / a;
    res.sqrt()
}

pub fn mass_to_hue(m: f32, min_mass: f32, max_mass: f32) -> f32 {
    // linear conversion
    // NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
    let new_max = 1.0;

    if min_mass == max_mass {
        return 1.0;
    }

    ((m - min_mass) * new_max) / (max_mass - min_mass)
}

fn spawn_bodies_from_settings(mut commands: Commands, settings: &SimulationSettings) {
    let norm_min = if settings.min_body_mass < settings.max_body_mass {
        settings.min_body_mass
    } else {
        settings.max_body_mass
    };

    let mut rng = rand::rng();
    let mut added_bodies = Vec::<(Body, Transform, Velocity)>::with_capacity(
        settings.n_bodies.try_into().unwrap_or(usize::MIN),
    );
    for _ in 0..settings.n_bodies {
        let rng_mass = rng.random_range(norm_min..=settings.max_body_mass);
        let body = Body {
            mass: rng_mass,
            radius: mass_to_radius(rng_mass),
            hue: mass_to_hue(rng_mass, settings.min_body_mass, settings.max_body_mass),
        };

        let transform: Transform;
        let velocity: Velocity;

        if settings.donut {
            let x = rng.random_range(settings.spawn_area.clone());
            let y = rng.random_range(settings.spawn_area.clone());
            let rng_mag = rng.random_range(10.0..=200.0);

            let dir = Vec2::new(x, y).normalize_or_zero();
            let rng_vec = dir * rng_mag;

            transform = Transform::from_xyz(rng_vec.x, rng_vec.y, settings.z);
            let angle_vel = Vec2::from(dir.perp() * settings.init_vel);
            velocity = Velocity(angle_vel);
        } else {
            let x = rng.random_range(settings.spawn_area.clone());
            let y = rng.random_range(settings.spawn_area.clone());

            transform = Transform::from_xyz(x, y, settings.z);
            velocity = Velocity(Vec2::ZERO);
        }

        added_bodies.push((body, transform, velocity));
    }
    commands.spawn_batch(added_bodies);
}

fn add_bodies(commands: Commands, settings: Res<SimulationSettings>) {
    spawn_bodies_from_settings(commands, &settings);
}

fn draw_bodies(
    mut commands: Commands,
    mut query: Query<(Entity, &Body), Added<Body>>,
    materials: Option<ResMut<Assets<ColorMaterial>>>,
    meshes: Option<ResMut<Assets<Mesh>>>,
) {
    // Spawn rendering materials only when available (to allow testing)
    if let (Some(mut meshes), Some(mut materials)) = (meshes, materials) {
        for (entity, body) in query.iter_mut() {
            commands
                .entity(entity)
                .insert(Mesh2d(meshes.add(Circle::new(body.radius))))
                .insert(MeshMaterial2d(
                    materials.add(ColorMaterial::from_color(Srgba::rgb(body.hue, 0.5, 0.0))),
                ));
        }
    }
}

fn build_quadtree(
    mut commands: Commands,
    query: Query<(Entity, &mut Body, &mut Transform, &mut Velocity)>,
) {
    let positions: Vec<Vec2> = query
        .iter()
        .map(|(_e, _b, t, _v)| Vec2::new(t.translation.x, t.translation.y))
        .collect();

    let quad = Quad::new_containing(&positions);
    // let quad = Quad::new(0.0, 0.0, 100000.0);
    let mut tree = Quadtree::new(quad);

    for (entity1, body1, transform1, _velocity1) in query.iter() {
        tree.insert(
            entity1,
            Vec2::new(transform1.translation.x, transform1.translation.y),
            *body1,
        );
    }

    commands.insert_resource(QuadtreeResource { tree });
}

fn compute_physics(
    mut query: Query<(Entity, &mut Body, &mut Transform, &mut Velocity)>,
    settings: Res<SimulationSettings>,
    tree_res: Res<QuadtreeResource>,
) {
    for (entity1, body1, transform1, mut velocity1) in query.iter_mut() {
        let pos2d = Vec2::new(transform1.translation.x, transform1.translation.y);
        let accel = tree_res.tree.get_total_accel(
            entity1,
            pos2d,
            *body1,
            settings.g,
            settings.delta_t,
            settings.theta,
        );
        velocity1.0 += accel;
    }
}

fn update_positions(
    mut query: Query<(&mut Transform, &Velocity)>,
    settings: Res<SimulationSettings>,
) {
    for (mut transform1, velocity) in query.iter_mut() {
        // velocity.0 += col_map.get(&entity1.index()).unwrap_or(&Vec3::ZERO);
        // velocity.0 += accel_map.get(&entity1.index()).unwrap();
        transform1.translation.x += velocity.0.x * settings.delta_t;
        transform1.translation.y += velocity.0.y * settings.delta_t;
    }
}

fn draw_tree(
    settings: Res<SimulationSettings>,
    tree_res: Res<QuadtreeResource>,
    gizmos: Option<Gizmos>,
) {
    if !settings.show_tree {
        return;
    }

    if let Some(gizmos) = gizmos {
        if settings.show_tree {
            tree_res.tree.draw_tree(gizmos);
        }
    }
}
