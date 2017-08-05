extern crate rand;
extern crate crossbeam;
use std::fs::File;
use std::io::BufWriter;
use std::io::Write;
mod vec3; //split into it's own file for readability
use vec3::*;
use std::thread::sleep;
use std::thread::spawn;
use std::time::{Duration, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize,Ordering};
use std::path::Path;
use std::f64;

extern crate tobj;

extern crate piston_window;
extern crate image;
use piston_window::{PistonWindow, WindowSettings, Texture, OpenGL, Transformed, clear, image as draw_image};
use piston_window::texture::TextureSettings;

#[derive(Copy, Clone)]
struct Ray {
    origin: Vec3,
    direction: Vec3,
}

#[derive(Copy, Clone)]
enum ReflType {
    DIFF,
    SPEC,
    REFR,
    LITE
}

#[derive(Copy, Clone)]
struct Material {
    emission: Vec3,
    color: Vec3,
    refl: ReflType,
}

struct Sphere {
    radius: f64,
    position: Vec3,
    material: Material,
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

fn clamp(x: f64) -> f64 {
    match x {
        _ if x < 0.0 => 0.0,
        _ if x > 1.0 => 1.0,
        _ => x
    }
}

fn to_int(x: f64) -> i32 {
    (f64::powf(clamp(x),1.0/2.2)*255.0 + 0.5) as i32
}

fn to_u8(x: f64) -> u8 {
    (f64::powf(clamp(x),1.0/2.2)*255.0 + 0.5) as u8
}

struct Triangle {
    points: [Vec3; 3],
}

impl Triangle {
    fn iter(&self) -> std::slice::Iter<Vec3> {
        self.points.iter()
    }

    fn normal(&self) -> Vec3 {
        (self.points[1]-self.points[0]).cross(self.points[2]-self.points[0])
    }
}

fn make_cube() -> Vec<Triangle> {
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
#[derive(Debug)]
struct AABB {
    x: (f64,f64),
    y: (f64, f64),
    z: (f64,f64),
}

impl AABB {
    fn split(&self, split_axis:Axis, split_val: f64) -> (AABB,AABB) {
        let mut lt = AABB {
            x: self.x,
            y: self.y,
            z: self.z,
        };
        let mut gt = AABB {
            x: self.x,
            y: self.y,
            z: self.z,
        };
        match split_axis {
            Axis::X => { lt.x.1 = split_val; gt.x.0 = split_val; },
            Axis::Y => { lt.y.1 = split_val; gt.y.0 = split_val; },
            Axis::Z => { lt.z.1 = split_val; gt.z.0 = split_val; },
        }
        (lt,gt)
    }
    fn contains_point(&self, point: &Vec3) -> bool {
        point.x > self.x.0 && point.x < self.x.1
            && point.y > self.y.0 && point.y < self.y.1
            && point.z > self.z.0 && point.z < self.z.1
    }
    fn contains_ray(&self, ray: Ray) -> bool {
        let mut tmin = -std::f64::INFINITY;
        let mut tmax = std::f64::INFINITY;
        let tx1 = (self.x.0 - ray.origin.x)/ray.direction.x;
        let tx2 = (self.x.1 - ray.origin.x)/ray.direction.x;
        tmin = f64::max(tmin, f64::min(tx1,tx2));
        tmax = f64::min(tmax, f64::max(tx1,tx2));
        let ty1 = (self.y.0 - ray.origin.y)/ray.direction.y;
        let ty2 = (self.y.1 - ray.origin.y)/ray.direction.y;
        tmin = f64::max(tmin, f64::min(ty1,ty2));
        tmax = f64::min(tmax, f64::max(ty1,ty2));
        let tz1 = (self.z.0 - ray.origin.z)/ray.direction.z;
        let tz2 = (self.z.1 - ray.origin.z)/ray.direction.z;
        tmin = f64::max(tmin, f64::min(tz1,tz2));
        tmax = f64::min(tmax, f64::max(tz1,tz2));
        tmax >= tmin

    }
    fn fminmax(iter: std::slice::Iter<Vec3>) -> (Vec3,Vec3) {
        let mut min = Vec3::new(std::f64::MIN);
        let mut max = Vec3::new(std::f64::MAX);
        for v in iter {
            if *v < min {
                min = *v;
            }
            if *v > max {
                max = *v;
            }
        }
        (min, max)
    }
    fn contains_tri(&self, tri: &Triangle) -> bool {
        //self.contains_point(&tri.points[0]) || self.contains_point(&tri.points[1]) || self.contains_point(&tri.points[2])
        
        let (start, end) = self.corners();
        let axes = [ Vec3::set(1.0,0.0,0.0),Vec3::set(0.0,1.0,0.0),Vec3::set(0.0,0.0,1.0) ];
        //Test the box normals
        for (axis,bounds) in axes.iter().zip([self.x,self.y,self.z].iter()) {
            //let min = dots.min().unwrap();
            //let max = dots.max().unwrap();
            let min = tri.points.iter().map(|x|axis.dot(*x)).fold(f64::INFINITY,f64::min);
            let max = tri.points.iter().map(|x|axis.dot(*x)).fold(f64::NEG_INFINITY,f64::max);
            if min > bounds.1 || max < bounds.0 {
                return false;
            }
        }
        //Test the triangle normal
        let offset = tri.normal().dot(tri.points[0]);
        let corners = self.all_corners();
        let min = corners.iter().map(|x|tri.normal().dot(*x)).fold(f64::INFINITY,f64::min);
        let max = corners.iter().map(|x|tri.normal().dot(*x)).fold(f64::NEG_INFINITY,f64::max);
        if  max < offset || min > offset {
            return false;
        }
        //test the edge cross products
        let edges = [ tri.points[0] - tri.points[1], tri.points[1] - tri.points[2], tri.points[2] - tri.points[0] ];
        for edge in edges.iter() {
            for axis in axes.iter() {
                let w = edge.cross(*axis);
                if corners.iter().map(|x|w.dot(*x)).fold(f64::NEG_INFINITY,f64::max) < tri.points.iter().map(|x|w.dot(*x)).fold(f64::INFINITY,f64::min) || corners.iter().map(|x|w.dot(*x)).fold(f64::INFINITY,f64::min) > tri.points.iter().map(|x|w.dot(*x)).fold(f64::NEG_INFINITY,f64::max) {
                    return false;
                }
            }
        }
        true
    }
    fn biggest_dim(&self, avg: Vec3) -> (Axis,f64) {
        let diff = Vec3::set(self.x.1,self.y.1,self.z.1) - Vec3::set(self.x.0,self.y.0,self.z.0);
        if diff.x > diff.y {
            if diff.x > diff.z {
                (Axis::X,avg.x)
            } else {
                (Axis::Z,avg.z)
            }
        } else {
            if diff.y > diff.z {
                (Axis::Y,avg.y)
            } else {
                (Axis::Z,avg.z)
            }
        }
    }
    fn split_biggest(&self, avg: Vec3) -> (AABB,AABB){
        let (biggest_axis, sval) = self.biggest_dim(avg);
        self.split(biggest_axis,sval)
    }

    pub fn valid(&self) -> bool {
        self.x.0 < self.x.1 && self.y.0 < self.y.1 && self.z.0 < self.z.1
    }

    pub fn corners(&self) -> (Vec3,Vec3) {
        (Vec3::set(self.x.0,self.y.0,self.z.0),Vec3::set(self.x.1,self.y.1,self.z.1))
    }

    pub fn all_corners(&self) -> [Vec3; 8] {
        let mut cnrs = [Vec3::zero(); 8];
        let mut idx = 0;
        for x in &[self.x.0,self.x.1] {
            for y in &[self.y.0,self.y.1] {
                for z in &[self.z.0,self.z.1] {
                    cnrs[idx] = Vec3::set(*x,*y,*z);
                }
            }
        }
        cnrs
    }
}

#[derive(Copy, Clone, Debug)]
enum Axis {
    X,
    Y,
    Z,
}

struct KDBranch {
    aabb: AABB,
    split_axis: Axis,
    split_val: f64,
    child_lt: Box<KDNode>,
    child_gt: Box<KDNode>,
}

struct KDLeaf {
    tri_inds: Vec<usize>,
}

enum KDNode {
    Branch(KDBranch),
    Leaf(KDLeaf),
}

struct KDTree {
    root: KDNode,
}

impl KDTree {
    fn biggest_dim(diff: Vec3, avg: Vec3) -> (Axis,f64) {
        if diff.x > diff.y {
            if diff.x > diff.z {
                (Axis::X,avg.x)
            } else {
                (Axis::Z,avg.z)
            }
        } else {
            if diff.y > diff.z {
                (Axis::Y,avg.y)
            } else {
                (Axis::Z,avg.z)
            }
        }
    }
    fn build_helper(triangles: &Vec<Triangle>, aabb:AABB, depth: usize) -> Box<KDNode> {
        let mut sum = Vec3::zero();
        let mut count:f64 = 0.0;
        let mut indices = Vec::new();
        for i in 0..triangles.len() {
            let ref tri = triangles[i];
            if aabb.contains_tri(tri) {
                indices.push(i);
                for pt in tri.iter() {
                    sum = sum + *pt;
                    count += 1.0;
                }
            }
        }
        let (biggest_axis, sval) = aabb.biggest_dim(sum * (1.0/count));
        let (la, ga) = aabb.split_biggest(sum * (1.0/count));
        //println!("count:{} sval:{} axis:{:?} la:{:?} ga:{:?}",count,sval,biggest_axis,la,ga);
        if depth < 3 && count > 18.0 && la.valid() && ga.valid() { //arbitrarily picked 3 triangles as leaf size
            Box::new(KDNode::Branch(KDBranch{
                aabb: aabb,
                split_axis: biggest_axis,
                split_val: sval,
                child_lt: KDTree::build_helper(triangles,la,depth+1),
                child_gt: KDTree::build_helper(triangles,ga,depth+1),
                }))
        } else {
            Box::new(KDNode::Leaf(KDLeaf {
                tri_inds: indices,
            }))
        }
    }

    pub fn build(triangles: &Vec<Triangle>) -> KDTree {
        let mut mins = Vec3::zero();
        let mut maxs = Vec3::zero();
        let mut sum = Vec3::zero();
        for tri in triangles {
            for pt in tri.iter() {
                mins.x = f64::min(mins.x,pt.x);
                mins.y = f64::min(mins.y,pt.y);
                mins.z = f64::min(mins.z,pt.z);
                maxs.x = f64::max(maxs.x,pt.x);
                maxs.y = f64::max(maxs.y,pt.y);
                maxs.z = f64::max(maxs.z,pt.z);
                sum = sum + *pt;
            }
        }
        let avg = sum * (1.0 / (triangles.len() * 3) as f64);
        let diff = maxs - mins;
        let (biggest_axis, sval) = KDTree::biggest_dim(diff, avg);
        let aabb = AABB {
            x:(mins.x,maxs.x),
            y:(mins.y,maxs.y),
            z:(mins.z,maxs.z),
        };
        let (la,ga) = aabb.split(biggest_axis,sval);
        KDTree {
            root: KDNode::Branch(KDBranch{
                aabb: aabb,
                split_axis: biggest_axis,
                split_val: sval,
            child_lt: KDTree::build_helper(triangles,la,0),
            child_gt: KDTree::build_helper(triangles,ga,0),
            }),
        }
    }

    fn intersect_helper<'a>(triangles: &'a Vec<Triangle>, ray: Ray, node: &KDNode,depth: usize) -> (Option<&'a Triangle>, f64){
        match *node {
            KDNode::Branch(ref branch) => {
                if !branch.aabb.contains_ray(ray) || depth > 10 {
                    return (None,std::f64::INFINITY);
                }
                if let (Some(x),y) = KDTree::intersect_helper(triangles, ray, &*branch.child_lt,depth+1) {
                    (Some(x),y)
                } else {
                    KDTree::intersect_helper(triangles, ray, &*branch.child_gt,depth+1)
                }
            },
            KDNode::Leaf(ref leaf) => {
                intersect_triangle(triangles,ray)
            },
        }
    }

    pub fn intersect<'a>(&self, triangles: &'a Vec<Triangle>, ray: Ray) -> (Option<&'a Triangle>, f64){
        KDTree::intersect_helper(triangles,ray,&self.root,0)
    }
}

//#[inline(always)]
fn intersect_triangle<'a>(triangles: &'a Vec<Triangle>, ray: Ray) -> (Option<&'a Triangle>, f64) {
    let inf = 1e20f64;
    let EPSILON = 0.000001f64;
    let mut depth = inf;
    let mut obj = Option::None;
    for triangle in triangles {
        let e1 = triangle.points[1] - triangle.points[0];
        let e2 = triangle.points[2] - triangle.points[0];
        let p = ray.direction.cross(e2);
        let det = e1.dot(p);
        if det > -EPSILON && det < EPSILON {
            continue;
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
fn intersect<'a>(spheres: &'a Vec<Sphere>, r: &'a Ray)  -> (Option< &'a Sphere>, f64) {
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
fn intersect_val<'a>(spheres: &'a Vec<Sphere>, r: Ray)  -> (Option< &'a Sphere>, f64) {
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

fn load_mesh(path: &str) -> (Vec<Triangle>, KDTree) {
    let (obj, mat) = tobj::load_obj(&Path::new(path)).unwrap();
    let obj_tris = obj.into_iter().flat_map(|x| {
        let mut tris = Vec::new();
        for f in 0..x.mesh.indices.len() / 3 {
            let mut tri = Vec::new();
            for ind in &x.mesh.indices[3*f..3*f+3] {
                let v = Vec3::set(x.mesh.positions[(3* *ind) as usize]as f64,
                    x.mesh.positions[(3* *ind) as usize +1]as f64,
                    x.mesh.positions[(3* *ind) as usize +2]as f64,
                );
                tri.push(v * 20.0 + Vec3::set(50.0,20.0,50.0));
            }
            tris.push(Triangle { points: [tri[0],tri[1],tri[2]]});
        }
        tris
    }).collect();
    let kdtree = KDTree::build(&obj_tris);
    (obj_tris, kdtree)
}

static PI: f64 = 3.14159265358979323;

#[allow(non_snake_case)] //allow variable naming here to match smallpt
fn radiance_iter(spheres: &Vec<Sphere>, mesh_data: (&Vec<Triangle>, &KDTree), ray: Ray) -> Vec3 {
    let mut depth: i32 = -1;
    let mut radiance = Vec3::new(0.0);
    let mut atten = Vec3::new(1.0);
    let mut r = ray;
    let (obj_tris, kdtree) = mesh_data;
    
    /*let cube:Vec<Triangle> = make_cube().into_iter().map(|x|Triangle { points:[
            x.points[0] * 20.0 + Vec3::set(50.0,20.0,50.0),
            x.points[1] * 20.0 + Vec3::set(50.0,20.0,50.0),
            x.points[2] * 20.0 + Vec3::set(50.0,20.0,50.0),
    ]}).collect();*/
    loop {
        depth = depth + 1;
        let (hit, dist) = intersect_val(spheres,r);
        //let (hit2, dist2) = intersect_triangle(&obj_tris, r);
        //let (hit2, dist2): (Option<Triangle>,f64) = (None,1e20f64);
        let (hit2, dist2) = kdtree.intersect(&obj_tris, r);
        //let hit2: Option<Triangle> = None; let dist2 = std::f64::INFINITY;

        let x = r.origin+ (r.direction*dist);
        let n;
        let mtl;
        if dist2 < dist {
            if let Some(triangle) = hit2 {
                //ReflType::DIFF
                let e1 = triangle.points[1] - triangle.points[0];
                let e2 = triangle.points[2] - triangle.points[0];
                let norm = e1.cross(e2);
                let r1 = 2.0 * PI * rand::random::<f64>();
                let r2 = rand::random::<f64>();
                let r2s = f64::sqrt(r2);
                n = norm.normalize().abs();
                mtl = Material {
                    emission: Vec3::zero(),
                    color: Vec3::set(0.25,0.75,0.25),
                    refl: ReflType::DIFF,
                }
            } else {
                break;
            }
        } else {
            if let Some(circle) = hit {
                let un = x - circle.position;
                n = un.normalize();
                mtl = circle.material;
            } else {
                break;
            }
        }
        let nl = if n.dot(r.direction) < 0.0 {
            n
        } else {
            n * -1.0
        };
        let mut f = mtl.color;
        let p = if f.x>f.y && f.x>f.z {
            f.x
        } else if f.y > f.z {
            f.y
        } else {
            f.z
        };
        depth +=1;
        if depth > 5 { //begin russian roulette after 5 bounces
                        //and terminate at 10 bounces
            if depth < 10 && rand::random::<f64>() < p {
                f = f * (1.0/p);
            } else {
                radiance = radiance + atten * mtl.emission;
                break;
            }
        }
        match mtl.refl {
            ReflType::LITE => {
                radiance = radiance + (atten * mtl.emission);
                break;
            },
            ReflType::DIFF => {
                let r1 = 2.0 * PI * rand::random::<f64>();
                let r2 = rand::random::<f64>();
                let r2s = f64::sqrt(r2);
                let w = nl;
                let u = if f64::abs(w.x) > 0.1 {
                    Vec3::set(0.0,1.0,0.0)
                } else {
                    Vec3::set(1.0,0.0,0.0)
                }.cross(w).normalize();
                let v = w.cross(u);
                let d = (u*f64::cos(r1)*r2s + v*f64::sin(r1)*r2s + w*f64::sqrt(1.0-r2)).normalize();
                //mtl.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                radiance = radiance + (atten * mtl.emission);
                atten = atten * f;
                r = Ray{origin:x,direction:d};
            },
            ReflType::SPEC => {
                let d = r.direction - n*2.0*n.dot(r.direction);
                //mtl.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                radiance = radiance + (atten * mtl.emission);
                atten = atten * f;
                r = Ray{origin:x,direction:d};
            },
            ReflType::REFR => {
                let d = r.direction - n*2.0*n.dot(r.direction);
                let reflRay = Ray { origin: x, direction: d };
                let into = n.dot(nl) > 0.0;
                let nc = 1f64;
                let nt = 1.5;
                let nnt;
                if into {
                    nnt = nc/nt;
                } else {
                    nnt = nt/nc;
                }
                let ddn = r.direction.dot(nl);
                let cos2t = 1.0 - nnt*nnt*(1.0-ddn*ddn);
                radiance = radiance + (atten * mtl.emission);
                if cos2t < 0.0 { //total internal reflection
                    //mtl.emission + f * radiance(spheres,&reflRay,depth)
                    atten = atten * f;
                    r = reflRay;
                } else {
                    let sign;
                    if into {
                        sign = 1.0;
                    } else {
                        sign = -1.0;
                    }
                    let tdir = (r.direction*nnt - n*(sign*(ddn*nnt+f64::sqrt(cos2t)))).normalize();
                    let a = nt-nc;
                    let b = nt+nc;
                    let R0 = a*a/(b*b);
                    let c;
                    if into {
                        c = 1.0-(-ddn);
                    } else {
                        c = 1.0 - tdir.dot(n);
                    }
                    let Re = R0+(1.0-R0)*c*c*c*c*c;
                    let Tr = 1.0-Re;
                    let P = 0.25+0.5*Re;
                    let RP = Re/P;
                    let TP = Tr/(1.0-P);
                    /*let tmp = //if *depth > 2 {
                        if rand::random::<f64>() < P { //Russian roulette
                            radiance(spheres,&reflRay,depth)*RP
                        } else {
                            radiance(spheres,&Ray { origin: x, direction: tdir },depth)*TP
                        }
                    } else {
                        radiance(spheres,&reflRay,depth)*Re + radiance(spheres,&Ray { origin: x, direction: tdir }, depth)*Tr
                    };
                    mtl.emission + f * tmp*/
                    //radiance = radiance + (atten * mtl.emission);
                    if rand::random::<f64>() < P {
                        atten = atten * RP;
                        r = reflRay;
                    } else {
                        atten = atten * TP * mtl.color;
                        r = Ray { origin: x, direction: tdir };
                    }
                }
            }
        }
    }
    radiance
}

#[allow(non_snake_case)] //allow variable naming here to match smallpt
fn radiance(spheres: &Vec<Sphere>, r: &Ray, depth: &mut i32) -> Vec3 {
    let (hit, dist) = intersect(spheres,r);
    match hit {
        None => Vec3::zero(), //we didn't intersect anything, return black
        Some(obj) => {
            let x = r.origin+ (r.direction*dist);
            let un = x - obj.position;
            let n = un.normalize();
            //check if we're inside the sphere or not
            let nl =  if n.dot(r.direction) < 0.0 {
                n
            } else {
                n * -1.0
            };
            let mut f = obj.material.color;
            let p = if f.x>f.y && f.x>f.z {
                f.x
            } else if f.y > f.z {
                f.y
            } else {
                f.z
            };
            *depth +=1;
            if *depth > 5 { //begin russian roulette after 5 bounces
                            //and terminate at 50 bounces
                if *depth < 50 && rand::random::<f64>() < p {
                    f = f * (1.0/p);
                } else {
                    return obj.material.emission;
                }
            }
            match obj.material.refl {
                ReflType::LITE => obj.material.emission,
                ReflType::DIFF => {
                    let r1 = 2.0 * PI * rand::random::<f64>();
                    let r2 = rand::random::<f64>();
                    let r2s = f64::sqrt(r2);
                    let w = nl;
                    let u = if f64::abs(w.x) > 0.1 {
                        Vec3::set(0.0,1.0,0.0)
                    } else {
                        Vec3::set(1.0,0.0,0.0)
                    }.cross(w).normalize();
                    let v = w.cross(u);
                    let d = (u*f64::cos(r1)*r2s + v*f64::sin(r1)*r2s + w*f64::sqrt(1.0-r2)).normalize();
                    obj.material.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                },
                ReflType::SPEC => {
                    let d = r.direction - n*2.0*n.dot(r.direction);
                    obj.material.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                },
                ReflType::REFR => {
                    let d = r.direction - n*2.0*n.dot(r.direction);
                    let reflRay = Ray { origin: x, direction: d };
                    let into = n.dot(nl) > 0.0;
                    let nc = 1f64;
                    let nt = 1.5;
                    let nnt;
                    if into {
                        nnt = nc/nt;
                    } else {
                        nnt = nt/nc;
                    }
                    let ddn = r.direction.dot(nl);
                    let cos2t = 1.0 - nnt*nnt*(1.0-ddn*ddn);
                    if cos2t < 0.0 { //total internal reflection
                        obj.material.emission + f * radiance(spheres,&reflRay,depth)
                    } else {
                        let sign;
                        if into {
                            sign = 1.0;
                        } else {
                            sign = -1.0;
                        }
                        let tdir = (r.direction*nnt - n*(sign*(ddn*nnt+f64::sqrt(cos2t)))).normalize();
                        let a = nt-nc;
                        let b = nt+nc;
                        let R0 = a*a/(b*b);
                        let c;
                        if into {
                            c = 1.0-(-ddn);
                        } else {
                            c = 1.0 - tdir.dot(n);
                        }
                        let Re = R0+(1.0-R0)*c*c*c*c*c;
                        let Tr = 1.0-Re;
                        let P = 0.25+0.5*Re;
                        let RP = Re/P;
                        let TP = Tr/(1.0-P);
                        let tmp = //if *depth > 2 {
                            if rand::random::<f64>() < P { //Russian roulette
                                radiance(spheres,&reflRay,depth)*RP
                            } else {
                                radiance(spheres,&Ray { origin: x, direction: tdir },depth)*TP
                            }
                        /*} else {
                            radiance(spheres,&reflRay,depth)*Re + radiance(spheres,&Ray { origin: x, direction: tdir }, depth)*Tr
                        };*/;
                        obj.material.emission + f * tmp
                    }
                }
            }
        }
    }
}

pub struct PathTracer {
    spheres: Arc<Vec<Sphere>>,
    mesh_data: (Vec<Triangle>, KDTree),
    width: usize,
    height: usize,
    samples: u32,
    buffer: Vec<Vec3>,
}

impl PathTracer {
    pub fn new(width: u32, height: u32) -> PathTracer {
        PathTracer {
            width: width as usize,
            height: height as usize,
            samples: 0,
            buffer: vec![Vec3::new(0.0);(width*height) as usize],
            mesh_data: load_mesh("cube.obj"),
            spheres: Arc::new(vec![
                Sphere { //left wall
                    radius: 1e5,
                    position: Vec3::set(1e5f64+1.0,40.8,81.6),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(0.75,0.25,0.25),
                        refl: ReflType::DIFF,
                    },
                },
                Sphere { //right wall
                    radius: 1e5,
                    position: Vec3::set(-1e5f64+99f64,40.8,81.6),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(0.25,0.25,0.75),
                        refl: ReflType::DIFF,
                    },
                },
                Sphere { //back wall
                    radius: 1e5,
                    position: Vec3::set(50.0,40.8,1e5f64),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(0.75,0.75,0.75),
                        refl: ReflType::DIFF,
                    },
                },
                Sphere { //front wall
                    radius: 1e5,
                    position: Vec3::set(50.0,40.8,-1e5f64+170f64),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::zero(),//Vec3::set(0.5,0.5,0.5),
                        refl: ReflType::DIFF,
                    },
                },
                Sphere { //floor
                    radius: 1e5,
                    position: Vec3::set(50.0,1e5f64,81.6),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(0.75,0.75,0.75),
                        refl: ReflType::DIFF,
                    },
                },
                Sphere { //ceiling
                    radius: 1e5,
                    position: Vec3::set(50.0,-1e5f64+81.6,81.6),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::new(0.75),//Vec3::set(0.75,0.75,0.75),
                        refl: ReflType::DIFF,
                    },
                },
                Sphere { //mirror ball
                    radius: 16.5,
                    position: Vec3::set(27.0,16.5,47.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::new(0.999),
                        refl: ReflType::SPEC,
                    },
                },
                Sphere { //glass ball
                    radius: 16.5,
                    position: Vec3::set(73.0,16.5,78.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::new(0.999),
                        refl: ReflType::REFR,
                    },
                },
                Sphere { //gold hemisphere
                    radius: 16.5,
                    position: Vec3::set(27.0,0.0,96.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(1.000, 0.766, 0.336),
                        refl: ReflType::SPEC,
                    },
                },
                Sphere { //green glass
                    radius: 8.0,
                    position: Vec3::set(50.0,8.0,110.0),
                    material: Material {
                        emission: Vec3::set(0.0,0.0,0.0),
                        color: Vec3::set(0.56,0.99,0.56),
                        refl: ReflType::REFR,
                    },
                },
                Sphere { //purple glass
                    radius: 8.0,
                    position: Vec3::set(8.0,8.0,110.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(0.75,0.50,1.0),
                        refl: ReflType::REFR,
                    },
                },
                Sphere { //purple glass
                    radius: 8.0,
                    position: Vec3::set(92.0,8.0,110.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(0.75,0.50,1.0),
                        refl: ReflType::REFR,
                    },
                },
                Sphere { //light
                    radius: 600.0,
                    position: Vec3::set(50.0,681.6-0.27,81.6),
                    material: Material {
                        emission: Vec3::new(12.0),
                        color: Vec3::zero(),
                        refl: ReflType::LITE,
                    },
                },
            ]),
        }
    }

    pub fn render(&mut self, samples_per_pass: u32) {
        let ref spheres = self.spheres;
        let (ref tris,ref kdtree) = self.mesh_data;
        let mesh_data = (tris,kdtree);
        let w = self.width;
        let h = self.height;
        self.samples += samples_per_pass;
        //println!("generating image {}x{} with {} samples per pixel",w,h,samps*4);//4 subsamples per pixel for AA
        let ref mut c = self.buffer;
        let progress = Arc::new(AtomicUsize::new(0));
        let start = Instant::now();
        crossbeam::scope(|scope| {
            let num_threads = 8;
            let section = h/num_threads;
            let chunk_length = w * section;
            let mut threads = Vec::with_capacity(num_threads);
            //println!("will process {} lines per thread with {} threads total",section,num_threads);
            for (thread,mut chunk) in c.chunks_mut(chunk_length).enumerate() {
                //create our per-thread variables that will be moved
                let cam = Ray { origin: Vec3::set(50.0,52.0,295.6), direction: Vec3::set(0.0,-0.042612,-1.0).normalize()};
                let cx = Vec3::set((w as f64)*0.5135/(h as f64),0.0,0.0);
                let cy = cx.cross(cam.direction).normalize()*0.5135;
                let my_spheres = spheres.clone();
                let thread_progress = progress.clone();
                let start = thread * section;
                threads.push(scope.spawn(move || {
                        for i in 0..section {
                            let y = h - (i + start);
                            for x in 0..w {
                                let mut pixel = Vec3::zero();
                                for sy in 0..2 {
                                    for sx in 0..2 {
                                        let mut r = Vec3::zero();
                                        for _ in 0..samples_per_pass {
                                            let r1 = 2.0*rand::random::<f64>();
                                            let dx = if r1 < 1.0 {
                                                f64::sqrt(r1)-1.0
                                            } else {
                                                1.0-f64::sqrt(2.0-r1)
                                            };
                                            let r2 = 2.0*rand::random::<f64>();
                                            let dy = if r2 < 1.0 {
                                                f64::sqrt(r2)-1.0
                                            } else {
                                                1.0-f64::sqrt(2.0-r2)
                                            };
                                            let d = cx * ( ( ((sx as f64)+0.5 + dx ) / 2.0 + (x as f64)) / (w as f64) - 0.5) +
                                                    cy * ( ( ((sy as f64)+0.5 + dy ) / 2.0 + (y as f64)) / (h as f64) - 0.5) + cam.direction;
                                            //r = r + radiance(&my_spheres, &Ray{origin:cam.origin+(d*140.0),direction:d.normalize()},&mut 0) * (1.0/(samps as f64));
                                            r = r + radiance_iter(&my_spheres, mesh_data, Ray{origin:cam.origin+(d*140.0),direction:d.normalize()});
                                        }
                                        pixel = pixel + (Vec3::set(r.x,r.y,r.z)*0.25);
                                    }
                                }
                                chunk[i*w + x] = chunk[i*w + x] + pixel;
                            }
                            thread_progress.fetch_add(1, Ordering::Relaxed);
                        }
                    }));
            }
            let mut p = 0;
            print!("\nprogress: 0.0%");
            while p < (h-1) {
                let np = progress.load(Ordering::Relaxed);
                if p != np {
                    let frac = p as f64 / (h-1) as f64;
                    print!("\rprogress: {:.1}%", frac * 100.0);
                    std::io::stdout().flush().unwrap();
                    p = np;
                }
                sleep(Duration::from_millis(50));
            }
            for t in threads {
                t.join();
            }
            //println!("\ndone!");
        });
        let duration = start.elapsed();
        let seconds = duration.as_secs() as f64 + (duration.subsec_nanos() as f64)/1e9f64;
        //println!("Time: {} seconds",seconds);
    }
}

pub fn main() {
    let opengl = OpenGL::V3_2;
    let width = 1440;//1024*2;
    let height = 1080;//768*2;
    let mut window: PistonWindow = WindowSettings::new(
            "rustpt",
            [width,height]
        )
        .opengl(opengl)
        .exit_on_esc(true)
        .build().unwrap();

    let mut tracer = PathTracer::new(width,height);
    let mut samples_per_pass = 1;
    println!("Rendering...");
    //tracer.render(samples_per_pass);
    let buffer = tracer.buffer.iter().flat_map(|pixel|vec![to_u8(pixel.x),to_u8(pixel.y),to_u8(pixel.z),255]).collect();
    let img = image::ImageBuffer::from_raw(width,height,buffer).unwrap();
    let mut texture = Texture::from_image(&mut window.factory,&img,&TextureSettings::new()).unwrap();
    let mut count = 1;
    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g| {
            print!("Pass {} with {} samples",count,samples_per_pass);
            tracer.render(samples_per_pass);
            count += 1;
            samples_per_pass = std::cmp::min(64,samples_per_pass * 2);
            clear([0.5, 0.0, 1.0, 1.0], g);
            let buffer = tracer.buffer.iter().flat_map(|pixel|{let p = *pixel * (1.0/(tracer.samples as f64)); vec![to_u8(p.x),to_u8(p.y),to_u8(p.z),255]}).collect();
            let img = image::ImageBuffer::from_raw(width,height,buffer).unwrap();
            texture.update(&mut g.encoder,&img).unwrap();
            //draw_image(&texture, c.transform.scale(2.0,2.0), g);
            //draw_image(&texture, c.transform.scale(0.5,0.5), g);
            draw_image(&texture, c.transform, g);
            let rgb_buffer: Vec<u8> = tracer.buffer.iter().flat_map(|pixel|{let p = *pixel * (1.0/(tracer.samples as f64)); vec![to_u8(p.x),to_u8(p.y),to_u8(p.z)]}).collect();
            println!("...Saved");
            spawn(move || {
                image::save_buffer(&Path::new("save.png"), &rgb_buffer, width, height, image::RGB(8)).unwrap();
            });
        });
    }
    
    /*let f = File::create("image.ppm").unwrap();
    let mut writer = BufWriter::new(f);
    write!(&mut writer,"P3\n{} {}\n{}\n",width,height,255).unwrap();
    for pixel in tracer.buffer {
        write!(&mut writer," {} {} {} ",to_int(pixel.x),to_int(pixel.y),to_int(pixel.z)).unwrap();
    }*/
    let rgb_buffer: Vec<u8> = tracer.buffer.iter().flat_map(|pixel|{let p = *pixel * (1.0/(tracer.samples as f64)); vec![to_u8(p.x),to_u8(p.y),to_u8(p.z)]}).collect();
    image::save_buffer(&Path::new("image.png"), &rgb_buffer, width, height, image::RGB(8)).unwrap();
}
