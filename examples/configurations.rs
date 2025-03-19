use bevy::input::common_conditions;
use bevy::prelude::*;
use bevy::render::renderer::RenderDevice;
use bevy_editor_cam::DefaultEditorCamPlugins;
use bevy_editor_cam::prelude::EditorCam;
use bevy_rapier3d::geometry::RapierColliderHandle;
use bevy_rapier3d::plugin::ReadRapierContext;
use bevy_rapier3d::prelude::{Collider, RigidBody};
use bevy_rapier3d::render::RapierDebugRenderPlugin;
use bevy_rich_text3d::{Text3d, Text3dBounds, Text3dPlugin, Text3dStyling, TextAtlas};
use bevy_wgsparkl::components::MpmCouplingEnabled;
use bevy_wgsparkl::resources::{AppState, PhysicsContext};
use nalgebra::{Vector3, vector};
use wgrapier3d::dynamics::body::{BodyCoupling, BodyCouplingEntry};
use wgsparkl3d::models::DruckerPrager;
use wgsparkl3d::solver::ParticlePhase;
use wgsparkl3d::{
    models::ElasticCoefficients,
    pipeline::MpmData,
    solver::{Particle, ParticleMassProps, SimulationParams},
};

pub fn main() {
    App::new()
        .add_plugins((DefaultPlugins, DefaultEditorCamPlugins))
        .add_plugins(bevy_rapier3d::plugin::RapierPhysicsPlugin::<()>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(bevy_wgsparkl::WgSparklPlugin)
        .add_plugins(Text3dPlugin {
            load_system_fonts: true,
            ..Default::default()
        })
        .add_systems(PostUpdate, setup_mpm_particles)
        .add_systems(
            Update,
            reset_scene.run_if(common_conditions::input_just_pressed(KeyCode::KeyR)),
        )
        .add_systems(Startup, setup_scene)
        .run();
}
pub fn setup_scene(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        EditorCam {
            last_anchor_depth: 110f64,
            ..Default::default()
        },
        Transform::from_xyz(0.0, 30.0, 100.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
    ));
    /*
     * Ground
     */
    let ground_size = 1200.1;
    let ground_height = 2.0;

    commands.spawn((
        Transform::from_xyz(0.0, -ground_height, 0.0),
        Collider::cuboid(ground_size, ground_height, ground_size),
        RigidBody::Fixed,
        MpmCouplingEnabled,
    ));
}

pub fn reset_scene(mut commands: Commands, mut app_state: ResMut<AppState>) {
    app_state.restarting = true;
    app_state.particles_initialized = false;
    commands.remove_resource::<PhysicsContext>();
}

pub fn setup_mpm_particles(
    mut commands: Commands,
    device: Res<RenderDevice>,
    mut app_state: ResMut<AppState>,
    rapier: ReadRapierContext,
    coupling: Query<&RapierColliderHandle, With<MpmCouplingEnabled>>,
    mut standard_materials: ResMut<Assets<StandardMaterial>>,
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

    let grid_size_x = 10;
    let grid_size_y = 10;
    let grid_size_z = 10;
    let num_particles = grid_size_x * grid_size_y * grid_size_z;

    let particle_positions = (0..num_particles)
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
        app_state.num_substeps = 16;
        app_state.gravity_factor = 1.0;
    };

    // Text material.
    let mat = standard_materials.add(StandardMaterial {
        base_color_texture: Some(TextAtlas::DEFAULT_IMAGE.clone_weak()),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..Default::default()
    });

    let params = SimulationParams {
        gravity: vector![0.0, -9.81, 0.0] * app_state.gravity_factor,
        dt: (1.0 / 60.0) / (app_state.num_substeps as f32),
    };

    let cell_width = 1f32;
    let mut particles = vec![];
    let mut configurations = vec![];

    let mut display_text_for_line = |z: f32, text: String| {
        commands.spawn((
            Text3d::new(text),
            Text3dStyling {
                size: 40.,
                color: Srgba::new(1., 1., 1., 1.),
                ..Default::default()
            },
            Text3dBounds { width: 500. },
            Mesh3d::default(),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(Vec3::new(
                -2f32 * grid_size_x as f32 * 3f32 * 2f32,
                5f32,
                z * grid_size_z as f32 * 0.7f32 * 3f32 * 2f32 + grid_size_z as f32 / 2f32,
            ))
            .with_scale(Vec3::splat(0.1))
            .with_rotation(Quat::from_rotation_x(-90f32.to_radians())),
        ));
    };
    {
        let young_modulus = 1_000_000_000.0;
        display_text_for_line(-1f32, format!("modulus = {}M", young_modulus / 1_000_000.0));
        // line with plasticity, varying poisson
        for x in -1..2 {
            let poisson_ratio = match x {
                -1 => 0.0,
                0 => 0.2,
                1 => 0.4,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            let plasticity = Some(DruckerPrager {
                h0: 35.0f32.to_radians(),
                h1: 9.0f32.to_radians(),
                h2: 0.2,
                h3: 10.0f32.to_radians(),
                ..DruckerPrager::new(model.lambda, model.mu)
            });
            configurations.push(ParticlesConfiguration {
                coords: IVec2::new(x, -1),
                density: 3700f32,
                model,
                plasticity,
                phase: None,
                description: format!("With plasticity.\n poisson: {}", poisson_ratio),
            });
        }
    }

    {
        let poisson_ratio = 0f32;
        display_text_for_line(0f32, format!("poisson = {}", poisson_ratio));
        // line with plasticity, varying young modulus
        for x in -1..2 {
            let young_modulus = match x {
                -1 => 1_000_000.0,
                0 => 10_000_000.0,
                1 => 100_000_000.0,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            let plasticity = Some(DruckerPrager {
                h0: 35.0f32.to_radians(),
                h1: 9.0f32.to_radians(),
                h2: 0.2,
                h3: 10.0f32.to_radians(),
                ..DruckerPrager::new(model.lambda, model.mu)
            });
            configurations.push(ParticlesConfiguration {
                coords: IVec2::new(x, 0),
                density: 3700f32,
                model,
                plasticity,
                phase: None,
                description: format!(
                    "With plasticity.\nmodulus: {}M",
                    young_modulus / 1_000_000f32
                ),
            });
        }
    }

    {
        let poisson_ratio = 0f32;
        display_text_for_line(1f32, format!("poisson = {}", poisson_ratio));
        // line without plasticity, varying young modulus
        for x in -1..2 {
            let young_modulus = match x {
                -1 => 1_000_000.0,
                0 => 50_000_000.0,
                1 => 200_000_000.0,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            configurations.push(ParticlesConfiguration {
                coords: IVec2::new(x, 1),
                density: 3700f32,
                model,
                plasticity: None,
                phase: Some(ParticlePhase {
                    phase: 1.0,
                    max_stretch: f32::MAX,
                }),
                description: format!(
                    "Without plasticity.\nmodulus: {}M",
                    young_modulus / 1_000_000.0
                ),
            });
        }
    }
    {
        let young_modulus = 1_000_000.0;
        display_text_for_line(2f32, format!("modulus = {}M", young_modulus / 1_000_000.0));
        // line without plasticity, varying poisson_ratio
        for x in -1..2 {
            let poisson_ratio = match x {
                -1 => -0.2f32,
                0 => 0.3,
                1 => 0.48,
                _ => unreachable!(),
            };
            let model = ElasticCoefficients::from_young_modulus(young_modulus, poisson_ratio);
            configurations.push(ParticlesConfiguration {
                coords: IVec2::new(x, 2),
                density: 3700f32,
                model,
                plasticity: None,
                phase: Some(ParticlePhase {
                    phase: 1.0,
                    max_stretch: f32::MAX,
                }),
                description: format!("Without plasticity.\npoisson: {}", poisson_ratio),
            });
        }
    }

    for c in configurations.iter() {
        let x = c.coords.x as f32 * 3f32;
        let z = c.coords.y as f32 * 3f32;
        let offset = vector![
            x * grid_size_x as f32,
            3f32,
            z * grid_size_z as f32 * 0.7f32
        ] * 2f32;
        for particle in &particle_positions {
            let position = vector![particle.x, particle.y, particle.z];

            let particle_size = vector![1.0, 1.0, 1.0];
            let volume = particle_size.x * particle_size.y * particle_size.z;
            let density = c.density;
            particles.push(Particle {
                position: nalgebra::Rotation::from_axis_angle(
                    &Vector3::z_axis(),
                    1f32.to_radians(),
                ) * vector![position.x, position.y, position.z]
                    + offset,
                velocity: Vector3::zeros(),
                volume: ParticleMassProps::new(density * volume, volume.cbrt() / 2.0),
                model: c.model,
                plasticity: c.plasticity,
                phase: c.phase,
            });
        }
        commands.spawn((
            Text3d::new(&c.description),
            Text3dStyling {
                size: 40.,
                color: Srgba::new(1., 1., 1., 1.),
                ..Default::default()
            },
            Text3dBounds { width: 500. },
            Mesh3d::default(),
            MeshMaterial3d(mat.clone()),
            Transform::from_translation(Vec3::new(
                offset.x,
                5f32,
                offset.z + 10f32 + grid_size_z as f32,
            ))
            .with_scale(Vec3::splat(0.1))
            .with_rotation(Quat::from_rotation_x(-90f32.to_radians())),
        ));
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

#[derive(Debug)]
pub struct ParticlesConfiguration {
    pub coords: IVec2,
    pub density: f32,
    pub model: ElasticCoefficients,
    pub plasticity: Option<DruckerPrager>,
    pub phase: Option<ParticlePhase>,
    pub description: String,
}
