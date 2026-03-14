use bevy::ecs::component::Component;


#[derive(Component, Clone, Copy)]
pub struct Body {
    pub mass: f32,
    pub radius: f32,
    pub hue: f32,
}