use std;
use std::ops::*;
use vec3::*;
use image::{DynamicImage,GenericImageView,Pixel};
use std::fs::File;


#[derive(Copy, Clone)]
pub enum ReflType {
    DIFF,
    SPEC,
    REFR,
    LITE,
    DebugNormal,
    DebugColor,
    PBR,
    PBR_REFR,
}

/*#[derive(Copy, Clone)]
pub struct Material {
    pub emission: Vec3,
    pub color: Vec3,
    pub refl: ReflType,
}*/
#[derive(Debug, Copy, Clone)]
pub enum Material {
    Diffuse(Vec3),
    Specular(Vec3),
    Refractive(Vec3),
    Light(Vec3),
    DebugNormal,
    DebugColor(Vec3),
    Physical(PhysicalMaterial)
}
#[derive(Debug, Copy, Clone)]
pub struct PhysicalMaterial {
    pub albedo: Vec3,
    pub metalness: f64,
    pub roughness: f64
}

#[derive(Default, Debug)]
pub struct PhysicalMaterialTextures {
    pub albedo: Option<Texture>,
    pub metalness: Option<Texture>,
    pub roughness: Option<Texture>
}

#[derive(Copy, Clone, Debug)]
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

#[derive(Debug,Copy,Clone,PartialEq)]
pub struct Triangle {
    pub points: [Vec3; 3],
}

impl Triangle {
    pub fn from_vec3_slice(slice: &[Vec3]) -> Triangle {
        Triangle {
            points: [
                slice[0],
                slice[1],
                slice[2]
            ]
        }
    }
    pub fn iter(&self) -> std::slice::Iter<Vec3> {
        self.points.iter()
    }

    pub fn normal(&self) -> Vec3 {
        (self.points[1]-self.points[0]).cross(self.points[2]-self.points[0])
    }
}

impl Add<Vec3> for Triangle {
    type Output = Triangle;
    fn add(self, rhs: Vec3) -> Triangle {
        Triangle {
            points: [
                self.points[0] + rhs,
                self.points[1] + rhs,
                self.points[2] + rhs,
            ]
        }
    }
}

impl Mul<Vec3> for Triangle {
    type Output = Triangle;
    fn mul(self, rhs: Vec3) -> Triangle {
        Triangle {
            points: [
                self.points[0] * rhs,
                self.points[1] * rhs,
                self.points[2] * rhs,
            ]
        }
    }
}

#[derive(Debug,Copy,Clone,PartialEq)]
pub struct MeshTriangle {
    pub pos: Triangle,
    pub texcoord: [[f32; 2]; 3],
    pub material_index: usize,
}

impl Add<Vec3> for MeshTriangle {
    type Output = MeshTriangle;
    fn add(self, rhs: Vec3) -> MeshTriangle {
        MeshTriangle {
            pos: self.pos + rhs,
            texcoord: self.texcoord,
            material_index: self.material_index
        }
    }
}

impl Mul<Vec3> for MeshTriangle {
    type Output = MeshTriangle;
    fn mul(self, rhs: Vec3) -> MeshTriangle {
        MeshTriangle {
            pos: self.pos * rhs,
            texcoord: self.texcoord,
            material_index: self.material_index
        }
    }
}

pub struct Texture {
    image: DynamicImage
}

impl Texture {
    pub fn from_path<P: AsRef<std::path::Path>>(path: P) -> image::ImageResult<Texture> {
        Ok(Texture::new(image::load(std::io::BufReader::new(File::open(path)?),image::ImageFormat::PNG)?))
    }
    pub fn new(image: image::DynamicImage) -> Texture {
        Texture {
            image
        }
    }
    pub fn index(&self, x: u32, y: u32) -> [u8;3] {
        //let start = (y * self.row_bytes) + (x * 3);
        //let slice = &self.data[start..(start + 3)];
        //[slice[0], slice[1], slice[2]]
        //[self.data[start], self.data[start+1], self.data[start+2]]
        let sx = x.min(self.image.width() - 1);
        let sy = y.min(self.image.height() - 1);
        unsafe {
            self.image.unsafe_get_pixel(sx,sy).to_rgb().data
        }
        //[x as u8,y as u8,255]
    }
    fn clamp(val: f32) -> f32 {
        val.max(0.0).min(1.0)
    }
    pub fn sample(&self, fx: f32, fy: f32) -> [u8;3] {
        let x = Self::clamp(fx) * (self.image.width() - 1) as f32;
        let y = Self::clamp(fy) * (self.image.height() - 1) as f32;
        let mut sum = [0u32;3];
        for i in &[x.floor(),x.ceil()] {
            for j in &[y.floor(),y.ceil()] {
                let pixel = self.index(*i as u32,*j as u32);
                for k in 0..3 {
                    sum[k] += pixel[k] as u32;
                }
            }
        }
        for i in 0..3 {
            sum[i] = sum[i] / 4;
        }
        [sum[0] as u8, sum[1] as u8, sum[2] as u8]
    }
}

impl std::fmt::Debug for Texture {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Texture {{ width: {}, height: {} }}", self.image.width(), self.image.height())
    }
}

#[inline(always)]
pub fn intersect_triangle<'a>(triangles: &'a [(Triangle,usize)], ray: Ray) -> (Option<&'a (Triangle,usize)>, f64, Vec3) {
    let epsilon = 0.00000001f64;
    let mut depth = std::f64::INFINITY;
    let mut obj = Option::None;
    let mut barycentric_u = 0.0;
    let mut barycentric_v = 0.0;
    for triangle in triangles {
        let e1 = triangle.0.points[1] - triangle.0.points[0];
        let e2 = triangle.0.points[2] - triangle.0.points[0];
        let p = ray.direction.cross(e2);
        let det = e1.dot(p);
        if det > -epsilon && det < epsilon {
            continue; //ray is parallel to the plane
        }
        let inv_det = 1.0 / det;
        let t = ray.origin - triangle.0.points[0];
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
        if potential_depth > epsilon && potential_depth < depth {
            obj = Option::Some(triangle);
            depth = potential_depth;
            barycentric_u = u;
            barycentric_v = v;
        }
    }
    (obj, depth, Vec3::set(barycentric_u,barycentric_v,1.0-(barycentric_u + barycentric_v)))
}

#[inline(always)]
pub fn intersect_selected_triangle<'a>(triangles: &'a [MeshTriangle], indices: &Vec<usize>, ray: Ray) -> (Option<&'a MeshTriangle>, f64, Vec3) {
    let epsilon = 0.00000001f64;
    let mut depth = std::f64::INFINITY;
    let mut obj = Option::None;
    let mut barycentric_u = 0.0;
    let mut barycentric_v = 0.0;
    for index in indices {
        let triangle = &triangles[*index];
        let e1 = triangle.pos.points[1] - triangle.pos.points[0];
        let e2 = triangle.pos.points[2] - triangle.pos.points[0];
        let p = ray.direction.cross(e2);
        let det = e1.dot(p);
        if det > -epsilon && det < epsilon {
            continue; //ray is parallel to the plane
        }
        let inv_det = 1.0 / det;
        let t = ray.origin - triangle.pos.points[0];
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
        if potential_depth > epsilon && potential_depth < depth {
            obj = Option::Some(triangle);
            depth = potential_depth;
            barycentric_u = u;
            barycentric_v = v;
        }
    }
    (obj, depth, Vec3::set(barycentric_u,barycentric_v,1.0-(barycentric_u + barycentric_v)))
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
