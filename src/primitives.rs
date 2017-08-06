use std;

use vec3::*;

#[derive(Copy, Clone)]
pub enum ReflType {
    DIFF,
    SPEC,
    REFR,
    LITE
}

#[derive(Copy, Clone)]
pub struct Material {
    pub emission: Vec3,
    pub color: Vec3,
    pub refl: ReflType,
}

#[derive(Copy, Clone)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
}

pub struct Sphere {
    pub radius: f64,
    pub position: Vec3,
    pub material: Material,
}

impl Sphere {
    #[inline(always)]
    pub fn intersect(&self, r: &Ray) -> Option<f64> {
        let op = self.position - r.origin; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        let eps = 1e-4f64;
        let b = op.dot(r.direction);
        let mut det = (b*b) + (self.radius * self.radius) - op.dot(op);
        if det < 0.0 {
            Option::None
        } else {
            det = f64::sqrt(det);
            let t = b - det;
            if t > eps {
                Option::Some(t)
            } else {
                let t = b + det;
                if t > eps {
                    Option::Some(t)
                } else {
                    Option::None
                }
            }
        }
    }
}

pub struct Triangle {
    pub points: [Vec3; 3],
}

impl Triangle {
    pub fn iter(&self) -> std::slice::Iter<Vec3> {
        self.points.iter()
    }

    pub fn normal(&self) -> Vec3 {
        (self.points[1]-self.points[0]).cross(self.points[2]-self.points[0])
    }
}

#[inline(always)]
pub fn intersect_triangle<'a>(triangles: &'a Vec<Triangle>, ray: Ray) -> (Option<&'a Triangle>, f64) {
    let inf = 1e20f64;
    let EPSILON = 0.00000001f64;
    let mut depth = inf;
    let mut obj = Option::None;
    for triangle in triangles {
        let e1 = triangle.points[1] - triangle.points[0];
        let e2 = triangle.points[2] - triangle.points[0];
        let p = ray.direction.cross(e2);
        let det = e1.dot(p);
        if det > -EPSILON && det < EPSILON {
            continue; //ray is parallel to the plane
        }
        let inv_det = 1.0 / det;
        let t = ray.origin - triangle.points[0];
        let u = t.dot(p) * inv_det;
        if u < 0.0 || u > 1.0 {
            continue;
        }
        let q = t.cross(e1);
        let v = ray.direction.dot(q) * inv_det;
        if v < 0.0 || (u + v) > 1.0 {
            continue;
        }
        let potential_depth = e2.dot(q) * inv_det;
        if potential_depth > EPSILON && potential_depth < depth {
            obj = Option::Some(triangle);
            depth = potential_depth;
        }
    }
    (obj, depth)
}

#[inline(always)]
pub fn intersect<'a>(spheres: &'a Vec<Sphere>, r: &'a Ray)  -> (Option< &'a Sphere>, f64) {
    let inf = 1e20f64;
    let mut t = inf;
    let mut obj = Option::None;
    for s in spheres {
        if let Some(d) = s.intersect(r) {
            if d < t {
                t = d;
                obj = Option::Some(s);
            }
        }
    }
    (obj, t)
}
#[inline(always)]
pub fn intersect_val<'a>(spheres: &'a Vec<Sphere>, r: Ray)  -> (Option< &'a Sphere>, f64) {
    let inf = 1e20f64;
    let mut t = inf;
    let mut obj = Option::None;
    for s in spheres {
        if let Some(d) = s.intersect(&r) {
            if d < t {
                t = d;
                obj = Option::Some(s);
            }
        }
    }
    (obj, t)
}

pub fn make_cube() -> Vec<Triangle> {
    vec![
        //front
        Triangle { points:[ 
            Vec3::set(-0.5, -0.5, -0.5),  
            Vec3::set( -0.5,  0.5, -0.5),  
            Vec3::set(  0.5,  0.5, -0.5),
        ]},
        Triangle { points:[ 
            Vec3::set( -0.5,  0.5, -0.5),  
            Vec3::set(  0.5,  0.5, -0.5),
            Vec3::set(  0.5, -0.5, -0.5),
        ]},  
        //back
        Triangle { points:[
            Vec3::set( 0.5, -0.5, 0.5 ),
            Vec3::set( 0.5,  0.5, 0.5 ),
            Vec3::set( -0.5,  0.5, 0.5 ),
        ]},
        Triangle { points:[
            Vec3::set( 0.5,  0.5, 0.5 ),
            Vec3::set( -0.5,  0.5, 0.5 ),
            Vec3::set( -0.5, -0.5, 0.5 ),
        ]},
        //RIGHT
        Triangle { points:[								
            Vec3::set( 0.5, -0.5, -0.5 ),
            Vec3::set( 0.5,  0.5, -0.5 ),
            Vec3::set( 0.5,  0.5,  0.5 ),
        ]},
        Triangle { points:[	
            Vec3::set( 0.5,  0.5, -0.5 ),
            Vec3::set( 0.5,  0.5,  0.5 ),
            Vec3::set( 0.5, -0.5,  0.5 ),
        ]},
        //LEFT
        Triangle { points:[	
            Vec3::set( -0.5, -0.5,  0.5 ),
            Vec3::set( -0.5,  0.5,  0.5 ),
            Vec3::set( -0.5,  0.5, -0.5 ),
        ]},
        Triangle { points:[	
            Vec3::set( -0.5,  0.5,  0.5 ),
            Vec3::set( -0.5,  0.5, -0.5 ),
            Vec3::set( -0.5, -0.5, -0.5 ),
        ]},
        //TOP				
        Triangle { points:[							 
            Vec3::set(  0.5,  0.5,  0.5 ),
            Vec3::set(  0.5,  0.5, -0.5 ),
            Vec3::set( -0.5,  0.5, -0.5 ),
        ]},
        Triangle { points:[	
            Vec3::set(  0.5,  0.5, -0.5 ),
            Vec3::set( -0.5,  0.5, -0.5 ),
            Vec3::set( -0.5,  0.5,  0.5 ),
        ]},
        //BOTTOM
        Triangle { points:[	
            Vec3::set(  0.5, -0.5, -0.5 ),
            Vec3::set(  0.5, -0.5,  0.5 ),
            Vec3::set( -0.5, -0.5,  0.5 ),
        ]},
        Triangle { points:[	
            Vec3::set(  0.5, -0.5,  0.5 ),
            Vec3::set( -0.5, -0.5,  0.5 ),
            Vec3::set( -0.5, -0.5, -0.5 ),
        ]},
    ]
}
