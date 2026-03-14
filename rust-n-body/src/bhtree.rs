use crate::Body;
use bevy::prelude::*;

pub struct Quadtree {
    root: TreeNode,
}

impl Quadtree {
    pub fn new(quad: Quad) -> Self {
        Quadtree {
            root: TreeNode::new(quad),
        }
    }

    pub fn insert(&mut self, entity: Entity, pos2d: Vec2, body: Body) {
        self.root.insert_into_subquad(entity, pos2d, body);
    }

    pub fn get_total_accel(
        &mut self,
        entity: Entity,
        pos2d: Vec2,
        body: Body,
        g: f32,
        dt: f32,
        theta: f32,
    ) -> Vec2 {
        // compute gravity * deltatime once per frame
        let k = g * dt;
        self.root
            .get_total_accel(entity, pos2d, body, k, theta)
    }
    pub fn draw_tree(&self, mut gizmos: Gizmos) {
        fn draw_node(node: &TreeNode, gizmos: &mut Gizmos) {
            gizmos.rect_2d(
                Isometry2d::from_xy(node.quad.center.x, node.quad.center.y),
                Vec2::splat(node.quad.size),
                Color::linear_rgba(0.0, 0.0, 1.0, 0.2),
            );
        }

        fn recurse(gizmos: &mut Gizmos, node: &TreeNode) {
            draw_node(&node, gizmos);

            if let Some(child) = &*node.children[0].node {
                recurse(gizmos, child);
            }
            if let Some(child) = &*node.children[1].node {
                recurse(gizmos, child);
            }
            if let Some(child) = &*node.children[2].node {
                recurse(gizmos, child);
            }
            if let Some(child) = &*node.children[3].node {
                recurse(gizmos, child);
            }
        }
        recurse(&mut gizmos, &self.root);
    }
}

struct TreeNode {
    quad: Quad,
    children: [Subquad; 4],
}

impl TreeNode {
    fn new(quad: Quad) -> Self {
        let h = quad.size / 2.0;
        let q = h / 2.0;

        TreeNode {
            quad,
            children: [
                Subquad::new(quad.center.x - q, quad.center.y + q, h),
                Subquad::new(quad.center.x + q, quad.center.y + q, h),
                Subquad::new(quad.center.x - q, quad.center.y - q, h),
                Subquad::new(quad.center.x + q, quad.center.y - q, h)
            ]
        }
    }

    fn insert_into_subquad(&mut self, entity: Entity, pos: Vec2, body: Body) {

        if !self.quad.contains(pos) {
            eprintln!(
                "\nPosition not found in any quads!: {:?}.\nTree node: \n{:?} \nQuads: \nnw: {:?} \nne: {:?} \nsw: {:?} \nse: {:?}",
                pos,
                &self.quad,
                &self.children[0].quad,
                &self.children[1].quad,
                &self.children[2].quad,
                &self.children[3].quad
            );
            return;
        }

        if pos.y < self.quad.center.y {
            // going south
            if pos.x < self.quad.center.x {
                // going west
                self.children[2].insert_or_divide(entity, pos, body);
            } else {
                // going east with ambiguous cases
                self.children[3].insert_or_divide(entity, pos, body);
            }
        } else {
            // going north
            if pos.x < self.quad.center.x {
                // going west
                self.children[0].insert_or_divide(entity, pos, body);
            } else {
                // going east with amb cases
                self.children[1].insert_or_divide(entity, pos, body);
            }
        }
    }

    fn get_total_accel(
        &self,
        entity: Entity,
        pos2d: Vec2,
        body: Body,
        k: f32,
        theta: f32,
    ) -> Vec2 {
        let mut cum_accel = Vec2::ZERO;

        cum_accel += get_accel(&self.children[0], entity, pos2d, body, k, theta);
        cum_accel += get_accel(&self.children[1], entity, pos2d, body, k, theta);
        cum_accel += get_accel(&self.children[2], entity, pos2d, body, k, theta);
        cum_accel += get_accel(&self.children[3], entity, pos2d, body, k, theta);

        cum_accel
    }
}

fn get_accel(
    subquad: &Subquad,
    entity: Entity,
    pos2d: Vec2,
    body: Body,
    k: f32,
    theta: f32,
) -> Vec2 {
    match &*subquad.node {
        None => {
            // Node is a leaf
            match subquad.entity {
                // With an occupant
                Some(tuple) => {
                    // But if the body is itself, no force should be exerted
                    if tuple.0.index() == entity.index() {
                        return Vec2::ZERO;
                    } else {
                        return calc_accel(
                            tuple.2.mass,
                            pos2d,
                            tuple.1,
                            k
                        );
                    }
                }
                None => {
                    // Nobody home;
                    return Vec2::ZERO;
                }
            }
        }
        Some(next_node) => {
            // Node is an internal node
            // S =  quad size
            // d = distance between node center of mass and body

            let s = subquad.quad.size;
            let d = pos2d.distance(next_node.quad.center);

            if s / d < theta {
                return calc_accel(subquad.mass, pos2d, subquad.pos_mass, k);
            } else {
                // node is too close to be treated as one. DIG DEEPER!!
                next_node.get_total_accel(entity, pos2d, body, k, theta)
            }
        }
    }
}

fn calc_accel(m2: f32, t1: Vec2, t2: Vec2, k: f32) -> Vec2 {
    const EPS2: f32 = 0.01;

    let r = t2 - t1;

    let dist_sq = r.length_squared() + EPS2;
    let inv_dist = dist_sq.sqrt().recip();
    let inv_dist2 = inv_dist * inv_dist;

    k * m2 * r * inv_dist2
}


struct Subquad {
    quad: Quad,
    entity: Option<(Entity, Vec2, Body)>,
    node: Box<Option<TreeNode>>,
    mass: f32,
    pos_mass: Vec2,
}

impl Subquad {
    fn new(x: f32, y: f32, size: f32) -> Self {
        Subquad {
            quad: Quad::new(x, y, size),
            entity: Option::None,
            node: Box::new(Option::None),
            mass: 0.0,
            pos_mass: Vec2 {
                x: 0.0,
                y: 0.0
            },
        }
    }

    fn insert_or_divide(&mut self, entity: Entity, pos: Vec2, body: Body) {
        match &mut *self.node {
            Some(node) => {
                // Node Is internal. Updat center of mass and total mass, and insert into subquadrants
                let m1 = self.mass;
                let m2 = body.mass;
                let m = m1 + m2;
                let x1 = self.pos_mass.x;
                let x2 = pos.x;
                let y1 = self.pos_mass.y;
                let y2 = pos.y;

                let x = (x1 * m1 + x2 * m2) / m;
                let y = (y1 * m1 + y2 * m2) / m;

                self.mass = m;
                self.pos_mass.x = x;
                self.pos_mass.y = y;

                node.insert_into_subquad(entity, pos, body);
            }
            None => {
                // Node is leaf. Insert if no body, or subdivide if occupied
                match self.entity {
                    None => {
                        // No body present
                        self.entity = Some((entity, pos, body));
                    }
                    Some(tuple) => {
                        // Node is occupied. We must dig deeper!!!1

                        if self.quad.size < 1.0 {
                            // unless node is too small.
                            // To avoid weird edge cases where it cannot be computed if a position is in a quad,
                            // we just add the mass and update center of mass of the node.
                            let m1 = self.mass;
                            let m2 = body.mass;
                            let m = m1 + m2;
                            let x1 = self.pos_mass.x;
                            let x2 = pos.x;
                            let y1 = self.pos_mass.y;
                            let y2 = pos.y;

                            let x = (x1 * m1 + x2 * m2) / m;
                            let y = (y1 * m1 + y2 * m2) / m;

                            self.mass = m;
                            self.pos_mass.x = x;
                            self.pos_mass.y = y;
                        } else {
                            let mut new_node = TreeNode::new(self.quad);

                            new_node.insert_into_subquad(entity, pos, body);
                            new_node.insert_into_subquad(tuple.0, tuple.1, tuple.2);

                            let m1 = self.mass;
                            let m2 = body.mass;
                            let m = m1 + m2;
                            let x1 = self.pos_mass.x;
                            let x2 = pos.x;
                            let y1 = self.pos_mass.y;
                            let y2 = pos.y;

                            let x = (x1 * m1 + x2 * m2) / m;
                            let y = (y1 * m1 + y2 * m2) / m;

                            self.mass = m;
                            self.pos_mass.x = x;
                            self.pos_mass.y = y;

                            self.node = Box::new(Some(new_node));
                        }
                    }
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Quad {
    center: Vec2,
    size: f32,
}

impl Quad {
    pub fn new(x: f32, y: f32, size: f32) -> Self {
        Quad {
            center: Vec2::new(x, y),
            size,
        }
    }

    pub fn new_containing(positions: &[Vec2]) -> Self {
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for p in positions {
            min_x = min_x.min(p.x);
            min_y = min_y.min(p.y);
            max_x = max_x.max(p.x);
            max_y = max_y.max(p.y);
        }

        let center = Vec2::new(min_x + max_x, min_y + max_y) * 0.5;
        let size = (max_x - min_x).max(max_y - min_y) + 1.0;

        Self { center, size }
    }

    fn contains(&self, pos: Vec2) -> bool {
        let hl = self.size / 2.0;
        let eps = hl * 0.0001;

        (self.center.x - pos.x).abs() - eps <= hl && (self.center.y - pos.y).abs() - eps <= hl
    }
}
