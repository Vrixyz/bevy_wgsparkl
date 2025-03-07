pub mod components;
pub mod instancing3d;
pub mod prep_vertex_buffer;
pub mod resources;
pub mod startup;
pub mod step;

use bevy::{asset::load_internal_asset, prelude::*};
use instancing3d::INSTANCING_SHADER_HANDLE;

pub struct WgSparklPlugin;

impl Plugin for WgSparklPlugin {
    fn build(&self, app: &mut App) {
        load_internal_asset!(
            app,
            INSTANCING_SHADER_HANDLE,
            "instancing3d.wgsl",
            Shader::from_wgsl
        );
        app.add_plugins(instancing3d::ParticlesMaterialPlugin);
        app.add_systems(Startup, startup::setup_app);
        app.add_systems(Update, step::step_simulation);
        app.add_systems(Update, startup::setup_graphics);
    }
}
