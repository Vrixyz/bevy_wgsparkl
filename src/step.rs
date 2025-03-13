use crate::instancing3d::InstanceMaterialData;
use crate::resources::{AppState, PhysicsContext, RunState, Timestamps};
use async_channel::{Receiver, Sender};
use bevy::prelude::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::tasks::ComputeTaskPool;
use bevy_rapier3d::plugin::{RapierContextMut, WriteRapierContext};
use wgcore::kernel::KernelInvocationQueue;
use wgcore::re_exports::encase::StorageBuffer;
use wgcore::timestamps::GpuTimestamps;
use wgsparkl3d::rapier::math::Vector;
use wgsparkl3d::wgparry::math::GpuSim;
use wgsparkl3d::wgrapier::dynamics::GpuVelocity;

#[derive(Resource)]
pub struct TimestampChannel {
    pub snd: Sender<Timestamps>,
    pub rcv: Receiver<Timestamps>,
}

#[allow(clippy::too_many_arguments)]
pub fn step_simulation(
    mut timings: ResMut<Timestamps>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    physics: Option<ResMut<PhysicsContext>>,
    mut app_state: ResMut<AppState>,
    mut rapier: WriteRapierContext,
    particles: Query<&InstanceMaterialData>,
    timings_channel: Res<TimestampChannel>,
) {
    if let Some(mut physics) = physics {
        step_simulation_multisteps(
            &mut timings,
            &render_device,
            &render_queue,
            &mut physics,
            &mut app_state,
            &mut rapier.single_mut(),
            &particles,
            &timings_channel,
        )
    }
}

#[allow(clippy::too_many_arguments)]
fn step_simulation_multisteps(
    timings: &mut Timestamps,
    render_device: &RenderDevice,
    render_queue: &RenderQueue,
    physics: &mut PhysicsContext,
    app_state: &mut AppState,
    rapier: &mut RapierContextMut,
    particles: &Query<&InstanceMaterialData>,
    timings_channel: &TimestampChannel,
) {
    if app_state.run_state == RunState::Paused {
        return;
    }

    let timings = &mut *timings;

    while let Ok(new_timings) = timings_channel.rcv.try_recv() {
        *timings = new_timings;
    }

    if let Some(t) = timings.timestamps.as_mut() {
        t.clear()
    }

    // Run the simulation.
    let device = render_device.wgpu_device();
    let physics = &mut *physics;
    let compute_queue = &*render_queue.0;
    let mut queue = KernelInvocationQueue::new(device);
    let mut encoder = device.create_command_encoder(&Default::default());

    // Send updated bodies information to the gpu.
    // PERF: don’t reallocate the buffers at each step.
    let poses_data: Vec<GpuSim> = physics
        .data
        .coupling()
        .iter()
        .map(|coupling| {
            let c = &rapier.colliders.colliders[coupling.collider];
            GpuSim::from_isometry(*c.position(), 1.0)
        })
        .collect();
    compute_queue.write_buffer(
        physics.data.bodies.poses().buffer(),
        0,
        bytemuck::cast_slice(&poses_data),
    );

    let gravity = Vector::y() * -9.81;
    let vels_data: Vec<_> = physics
        .data
        .coupling()
        .iter()
        .map(|coupling| {
            let rb = &rapier.rigidbody_set.bodies[coupling.body];
            GpuVelocity {
                linear: *rb.linvel()
                    + gravity
                        * rapier.simulation.integration_parameters.dt
                        * (rb.is_dynamic() as u32 as f32)
                        / (app_state.num_substeps as f32),
                angular: *rb.angvel(),
            }
        })
        .collect();
    let mut vels_bytes = vec![];
    let mut buffer = StorageBuffer::new(&mut vels_bytes);
    buffer.write(&vels_data).unwrap();
    compute_queue.write_buffer(physics.data.bodies.vels().buffer(), 0, &vels_bytes);

    //// Step the simulation.
    app_state
        .pipeline
        .queue_step(&mut physics.data, &mut queue, timings.timestamps.is_some());

    for _ in 0..app_state.num_substeps {
        queue.encode(&mut encoder, timings.timestamps.as_mut());
    }
    physics
        .data
        .poses_staging
        .copy_from(&mut encoder, physics.data.bodies.poses());
    if let Some(t) = timings.timestamps.as_mut() {
        t.resolve(&mut encoder)
    }

    // Prepare the vertex buffer for rendering the particles.
    if let Ok(instances_buffer) = particles.get_single() {
        queue.clear();
        app_state.prep_vertex_buffer.queue(
            &mut queue,
            &app_state.gpu_render_config,
            &physics.data.particles,
            &physics.data.grid,
            &physics.data.sim_params,
            &instances_buffer.buffer.buffer,
        );
        queue.encode(&mut encoder, timings.timestamps.as_mut());
    }

    // Submit.
    compute_queue.submit(Some(encoder.finish()));

    // let new_poses = futures::executor::block_on(physics.data.poses_staging.read(device)).unwrap();
    //
    // for (i, (_, rb)) in rapier.bodies.iter_mut().enumerate() {
    //     if rb.is_dynamic() {
    //         let vel_before = *rb.linvel();
    //         let interpolator = RigidBodyPosition {
    //             position: *rb.position(),
    //             next_position: new_poses[i].isometry,
    //         };
    //         let vel = interpolator.interpolate_velocity(
    //             1.0 / (rapier.integration_parameters.dt / divisor),
    //             &rb.mass_properties().local_mprops.local_com,
    //         );
    //         rb.set_linvel(vel.linvel, true);
    //         rb.set_angvel(vel.angvel, true);
    //         println!("dvel: {:?}", vel.linvel - vel_before);
    //     }
    // }

    if let Some(timestamps) = std::mem::take(&mut timings.timestamps) {
        let timings_snd = timings_channel.snd.clone();
        let timestamp_period = compute_queue.get_timestamp_period();
        let num_substeps = app_state.num_substeps;
        let timestamps_future = async move {
            let values = timestamps.wait_for_results_async().await.unwrap();
            let timestamps_ms = GpuTimestamps::timestamps_to_ms(&values, timestamp_period);
            let mut new_timings = Timestamps {
                timestamps: Some(timestamps),
                ..Default::default()
            };
            // `GpuTimestamps` uses a buffer of 2 `Timestamps`, one for the start and one for the end of the operation,
            // it's holding 9 floats (see `timings` below).
            debug_assert!(
                timestamps_ms.len() >= num_substeps * 2 * 9,
                "GpuTimestamps should be initialized with a bigger size"
            );
            for i in 0..num_substeps {
                let mut timings = [
                    &mut new_timings.grid_sort,
                    &mut new_timings.grid_update_cdf,
                    &mut new_timings.p2g_cdf,
                    &mut new_timings.g2p_cdf,
                    &mut new_timings.p2g,
                    &mut new_timings.grid_update,
                    &mut new_timings.g2p,
                    &mut new_timings.particles_update,
                    &mut new_timings.integrate_bodies,
                ];
                let start_index = i * timings.len() * 2;
                let times = &timestamps_ms[start_index..];

                for (k, timing) in timings.iter_mut().enumerate() {
                    **timing += times[k * 2 + 1] - times[k * 2];
                }
            }
            timings_snd.send(new_timings).await.unwrap();
        };

        ComputeTaskPool::get().spawn(timestamps_future).detach();
    }

    if app_state.run_state == RunState::Step {
        app_state.run_state = RunState::Paused;
    }
}
