use crate::Body;
use bevy::prelude::*;

pub struct Quadtree {
    nodes: Vec<TreeNode>,
    root: usize,
}

impl Quadtree {
    pub fn new(quad: Quad) -> Self {
        Quadtree {
            nodes: vec![TreeNode::new(quad)],
            root: 0,
        }
    }

    fn alloc_node(&mut self, node: TreeNode) -> usize {
        let index = self.nodes.len();
        self.nodes.push(node);
        index
    }

    pub fn insert(&mut self, entity: Entity, pos2d: Vec2, body: Body) {
        self.insert_into_subquad(self.root, entity, pos2d, body);
    }

    fn insert_into_subquad(&mut self, node_index: usize, entity: Entity, pos: Vec2, body: Body) {
        let node = &mut self.nodes[node_index];

        if !node.quad.contains(pos) {
            eprintln!(
                "\nPosition not found in any quads!: {:?}.\nTree node: \n{:?} \nQuads: \nnw: {:?} \nne: {:?} \nsw: {:?} \nse: {:?}",
                pos,
                &node.quad,
                &node.children[0].quad,
                &node.children[1].quad,
                &node.children[2].quad,
                &node.children[3].quad
            );
            return;
        }

        let subquad_index = self.choose_subquad(node_index, pos);

        if let Some((next_node, occupant)) =
            self.insert_or_divide(node_index, subquad_index, entity, pos, body)
        {
            if let Some(tuple) = occupant {
                self.insert_into_subquad(next_node, tuple.0, tuple.1, tuple.2);
            }

            self.insert_into_subquad(next_node, entity, pos, body);
        }
    }

    fn choose_subquad(&self, node_index: usize, pos: Vec2) -> usize {
        let node = &self.nodes[node_index];

        if pos.y < node.quad.center.y {
            // going south
            if pos.x < node.quad.center.x {
                // going west
                return 2;
            } else {
                // going east with ambiguous cases
                return 3;
            }
        } else {
            // going north
            if pos.x < node.quad.center.x {
                // going west
                return 0;
            } else {
                // going east with amb cases
                return 1;
            }
        }
    }

    fn insert_or_divide(
        &mut self,
        node_index: usize,
        subquad_index: usize,
        entity: Entity,
        pos: Vec2,
        body: Body,
    ) -> Option<(usize, Option<(Entity, Vec2, Body)>)> {
        let mut occupant = None;
        let mut new_node = None;

        // Borrow scope for self
        {
            let node = &mut self.nodes[node_index];
            let subquad = &mut node.children[subquad_index];

            match &mut subquad.node_index {
                Some(node_index) => {
                    // Node Is internal. Updat center of mass and total mass, and insert into subquadrants
                    let m1 = subquad.mass;
                    let m2 = body.mass;
                    let m = m1 + m2;
                    let x1 = subquad.pos_mass.x;
                    let x2 = pos.x;
                    let y1 = subquad.pos_mass.y;
                    let y2 = pos.y;

                    let x = (x1 * m1 + x2 * m2) / m;
                    let y = (y1 * m1 + y2 * m2) / m;

                    subquad.mass = m;
                    subquad.pos_mass.x = x;
                    subquad.pos_mass.y = y;

                    // Return index of occpying node
                    return Some((*node_index, None));
                }
                None => {
                    // Node is leaf. Insert if no body, or subdivide if occupied
                    match subquad.entity {
                        None => {
                            // No body present
                            subquad.entity = Some((entity, pos, body));
                            return None;
                        }
                        Some(tuple) => {
                            // Node is occupied. We must dig deeper!!!1

                            if subquad.quad.size < 1.0 {
                                // unless node is too small.
                                // To avoid weird edge cases where it cannot be computed if a position is in a quad,
                                // we just add the mass and update center of mass of the node.
                                let m1 = subquad.mass;
                                let m2 = body.mass;
                                let m = m1 + m2;
                                let x1 = subquad.pos_mass.x;
                                let x2 = pos.x;
                                let y1 = subquad.pos_mass.y;
                                let y2 = pos.y;

                                let x = (x1 * m1 + x2 * m2) / m;
                                let y = (y1 * m1 + y2 * m2) / m;

                                subquad.mass = m;
                                subquad.pos_mass.x = x;
                                subquad.pos_mass.y = y;
                                return None;
                            }

                            // self.insert_into_subquad(new_index, entity, pos, body);
                            // self.insert_into_subquad(new_index, tuple.0, tuple.1, tuple.2);

                            let m1 = subquad.mass;
                            let m2 = body.mass;
                            let m = m1 + m2;
                            let x1 = subquad.pos_mass.x;
                            let x2 = pos.x;
                            let y1 = subquad.pos_mass.y;
                            let y2 = pos.y;

                            let x = (x1 * m1 + x2 * m2) / m;
                            let y = (y1 * m1 + y2 * m2) / m;

                            subquad.mass = m;
                            subquad.pos_mass.x = x;
                            subquad.pos_mass.y = y;

                            new_node = Some(TreeNode::new(subquad.quad));
                            occupant = Some(tuple);
                        }
                    }
                }
            }
        } // Borrow scope ends

        if let Some(new_node) = new_node {
            let new_index = self.alloc_node(new_node);

            let node = &mut self.nodes[node_index];
            node.children[subquad_index].node_index = Some(new_index);

            return Some((new_index, occupant));
        }
        None
    }

    pub fn get_total_accel(
        &self,
        entity: Entity,
        pos2d: Vec2,
        body: Body,
        g: f32,
        dt: f32,
        theta: f32,
    ) -> Vec2 {
        // compute gravity * deltatime once per frame
        let k = g * dt;
        self.traverse_node(self.root, entity, pos2d, body, k, theta)
    }

    fn traverse_node(
        &self,
        node_index: usize,
        entity: Entity,
        pos2d: Vec2,
        body: Body,
        k: f32,
        theta: f32,
    ) -> Vec2 {
        let mut cum_accel = Vec2::ZERO;
        let node = &self.nodes[node_index];

        for subquad in &node.children {
            cum_accel += self.eval_subquad(subquad, entity, pos2d, body, k, theta);
        }

        cum_accel
    }

    fn eval_subquad(
        &self,
        subquad: &Subquad,
        entity: Entity,
        pos2d: Vec2,
        body: Body,
        k: f32,
        theta: f32,
    ) -> Vec2 {
        match &subquad.node_index {
            None => {
                // Node is a leaf
                match subquad.entity {
                    // With an occupant
                    Some(tuple) => {
                        // But if the body is itself, no force should be exerted
                        if tuple.0.index() == entity.index() {
                            return Vec2::ZERO;
                        }
                        return calc_accel(tuple.2.mass, pos2d, tuple.1, k);
                    }
                    None => {
                        // Nobody home;
                        return Vec2::ZERO;
                    }
                }
            }
            Some(next_node_index) => {
                // Node is an internal node
                // S =  quad size
                // d = distance between node center of mass and body

                let s = subquad.quad.size;
                let d = pos2d.distance(subquad.pos_mass);

                if s / d < theta {
                    return calc_accel(subquad.mass, pos2d, subquad.pos_mass, k);
                }
                // node is too close to be treated as one. DIG DEEPER!!
                self.traverse_node(*next_node_index, entity, pos2d, body, k, theta)
            }
        }
    }

    pub fn draw_tree(&self, mut gizmos: Gizmos) {
        fn draw_node(node: &TreeNode, gizmos: &mut Gizmos) {
            gizmos.rect_2d(
                Isometry2d::from_xy(node.quad.center.x, node.quad.center.y),
                Vec2::splat(node.quad.size),
                Color::linear_rgba(0.0, 0.0, 1.0, 0.2),
            );
        }

        for node in &self.nodes {
            draw_node(&node, &mut gizmos);
        }
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
                Subquad::new(quad.center.x + q, quad.center.y - q, h),
            ],
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
    node_index: Option<usize>,
    mass: f32,
    pos_mass: Vec2,
}

impl Subquad {
    fn new(x: f32, y: f32, size: f32) -> Self {
        Subquad {
            quad: Quad::new(x, y, size),
            entity: Option::None,
            node_index: Option::None,
            mass: 0.0,
            pos_mass: Vec2 { x: 0.0, y: 0.0 },
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
