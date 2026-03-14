use bevy::prelude::*;
use crate::{Body, Velocity, SimulationSettings};

// Check collisions with bodies and update their velocities?
// The velocity of a body is a vector that represents its speed and direction in 3D space (or 2D space)
pub fn collision(
    
    // Query for entities with Transform, Velocity, and Body components
    // The Transform component contains the position and rotation of the entity
    // The Velocity component contains the velocity of the entity
    // The Body component contains the mass and radius of the entity
    // A query is a way to access entities and their components in Bevy

    mut bodies: Query<(Entity, &mut Transform, &mut Velocity, &Body)>,
    settings: Res<SimulationSettings>, // SimulationSettings contains the simulation settings defined in main.rs
) {

    // Checking to see if collision detection is enabled, if its not, then it should just return
    if !settings.collision_enabled {
        return; 
    }

    // Get the elasticity value from the simulation settings which is user defined
    let elasticity = settings.elasticity;

    // This holds all the entities that are in the simulation
    // let mut items: Vec<_> = bodies.iter_mut().collect();
    let mut items: Vec<(Entity, Mut<Transform>, Mut<Velocity>, &Body)> = bodies.iter_mut().collect();

    // We loop through all the items in the simulation
    for i in 0..items.len() {
        // We need to split the items into two parts
        // Because of rust borrow checker rules, Body A and Body B cannot be borrowed with the same mutable reference
        // So we split the items into two parts, left and right
        // Left part gets the current item index plus the next item and the right part gets the rest of the items

        let split = i + 1;
        let (left, right) = items.split_at_mut(split);
        let (_entity_a, transform_a, velocity_a, body_a) = &mut left[i]; 

        // We loop throught the right part of the items split
        for li in 0..right.len() {
            let (_entity_b, transform_b, velocity_b, body_b) = &mut right[li];

            // We get the 2d positions of the two bodies
            let position_a = Vec2::new(transform_a.translation.x, transform_a.translation.y);
            let position_b = Vec2::new(transform_b.translation.x, transform_b.translation.y);

            // We get the distance between the two bodies
            let distance = position_a.distance(position_b);

            let min_distance = body_a.radius + body_b.radius; // If the sum of radius is bigger than the distance, then they are colliding

                // Check if the distance is less than the minimum distance
                if distance < min_distance {
                    let normal = (position_b - position_a).normalize();
                    let relative_velocity = velocity_b.0 - velocity_a.0;
                    let velocity_along_normal = relative_velocity.dot(normal);

                    if velocity_along_normal > 0.0 {
                        continue;
                    }

                    let impulse_magnitude = -(1.0 + elasticity) * velocity_along_normal
                        / (1.0 / body_a.mass + 1.0 / body_b.mass);

                    let impulse = impulse_magnitude * normal;

                    velocity_a.0 -= impulse / body_a.mass;
                    velocity_b.0 += impulse / body_b.mass;
                }
            }
    }
}

