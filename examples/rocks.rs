use bevy::prelude::*;
use bevy::render::renderer::RenderDevice;
use bevy_rapier3d::geometry::RapierColliderHandle;
use bevy_rapier3d::plugin::ReadRapierContext;
use bevy_rapier3d::prelude::{Collider, RigidBody};
use bevy_rapier3d::render::RapierDebugRenderPlugin;
use bevy_wgsparkl::components::MpmCouplingEnabled;
use bevy_wgsparkl::resources::{AppState, PhysicsContext};
use nalgebra::{Vector3, vector};
use wgrapier3d::dynamics::body::{BodyCoupling, BodyCouplingEntry};
use wgsparkl3d::models::DruckerPrager;
use wgsparkl3d::{
    models::ElasticCoefficients,
    pipeline::MpmData,
    solver::{Particle, ParticleMassProps, SimulationParams},
};

pub fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(bevy_rapier3d::plugin::RapierPhysicsPlugin::<()>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(bevy_wgsparkl::WgSparklPlugin)
        .add_systems(PostUpdate, setup_mpm_particles)
        .add_systems(Startup, setup_scene)
        .run();
}
pub fn setup_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-30.0, 30.0, 100.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
    ));
    /*
     * Ground
     */
    let ground_size = 200.1;
    let ground_height = 2.0;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
        RigidBody::Fixed,
        MpmCouplingEnabled,
    ));
}

pub fn setup_mpm_particles(
    mut commands: Commands,
    device: Res<RenderDevice>,
    mut app_state: ResMut<AppState>,
    rapier: ReadRapierContext,
    coupling: Query<&RapierColliderHandle, With<MpmCouplingEnabled>>,
) {
    if rapier.rapier_context.get_single().is_err() {
        return; // Rapier isn’t initialized yet.
    }

    let rapier = rapier.single();

    if rapier.colliders.colliders.is_empty() {
        return; // Rapier isn’t initialized yet.
    }

    if app_state.particles_initialized {
        return; // Already initialized.
    }

    let grid_size_x = 25;
    let grid_size_y = 25;
    let grid_size_z = 25;
    let num_rocks = grid_size_x * grid_size_y * grid_size_z;

    let rocks = (0..num_rocks)
        .into_iter()
        .map(|i| {
            let x = i % grid_size_x;
            let y = (i / grid_size_x) % grid_size_y;
            let z = (i / (grid_size_x * grid_size_y)) % grid_size_z;
            Vector3::new(x as f32, y as f32 + 1f32, z as f32)
        })
        .collect::<Vec<_>>();

    app_state.particles_initialized = true;

    let coupling: Vec<_> = coupling
        .iter()
        .map(|co_handle| {
            let co = &rapier.colliders.colliders[co_handle.0];
            println!("Coupled collider: {:?}", co.shape().shape_type());
            println!(
                "Coupled collider pose: {:?}",
                co.position().translation.vector
            );
            let rb_handle = co.parent().unwrap();
            BodyCouplingEntry {
                body: rb_handle,
                collider: co_handle.0,
                mode: BodyCoupling::OneWay, // TODO: try out two-ways for the particles to affect the rigid bodies.
            }
        })
        .collect();

    let device = device.wgpu_device();

    if !app_state.restarting {
        app_state.num_substeps = 32;
        app_state.gravity_factor = 1.0;
    };

    let params = SimulationParams {
        gravity: vector![0.0, -9.81, 0.0] * app_state.gravity_factor,
        dt: (1.0 / 60.0) / (app_state.num_substeps as f32),
    };

    let cell_width = 1.0;
    let mut particles = vec![];

    for rock in &rocks {
        let position = vector![rock.x, rock.y, rock.z];

        let rock_size = vector![1.0, 1.0, 1.0];
        let volume = rock_size.x * rock_size.y * rock_size.z;
        let density = 700.0;
        particles.push(Particle {
            position: vector![position.x, position.y, position.z],
            velocity: Vector3::zeros(),
            volume: ParticleMassProps::new(density * volume, volume.cbrt() / 2.0),
            model: ElasticCoefficients::from_young_modulus(10_000_000.0, 0.2),
            plasticity: Some(DruckerPrager {
                // TODO: tune these values.
                h0: 45.0f32.to_radians(),
                h1: 50.0f32.to_radians(),
                h2: 0.4,
                h3: 15.0f32.to_radians(),
                ..DruckerPrager::new(10_000_000.0, 0.2)
            }),
            phase: None,
        });
    }

    println!("Number of simulated particles: {}", particles.len());

    println!("Coupled: {}", coupling.len());

    let data = MpmData::with_select_coupling(
        device,
        params,
        &particles,
        &rapier.rigidbody_set.bodies,
        &rapier.colliders.colliders,
        coupling,
        cell_width,
        60_000,
    );
    commands.insert_resource(PhysicsContext { data, particles });
}
