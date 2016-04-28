extern crate rand;
use std::fs::File;
use std::io::BufWriter;
use std::io::Write;
mod vec3; //split into it's own file for readability
use vec3::*;
use std::thread::{spawn,sleep_ms};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize,Ordering};

struct Ray {
    origin: Vec3,
    direction: Vec3,
}

impl Ray {
    pub fn new(o: Vec3,d: Vec3) -> Ray {
        Ray { origin: o, direction: d }
    }
}

enum ReflType {
    DIFF,
    SPEC,
    REFR
}

struct Sphere {
    radius: f64,
    position: Vec3,
    emission: Vec3,
    color: Vec3,
    refl: ReflType,
}

impl Sphere {
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
    match(x){
        _ if x < 0.0 => 0.0,
        _ if x > 1.0 => 1.0,
        _ => x//(1.0 * x) / ((1.0 * x) + 1.0)
    }
}

fn toInt(x: f64) -> i32 {
    (f64::powf(clamp(x),1.0/2.2)*255.0 + 0.5) as i32
}

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

static PI: f64 = 3.14159265358979323;

#[allow(non_snake_case)] //allow variable naming here to match smallpt
fn radiance(spheres: &Vec<Sphere>, r: &Ray, depth: &mut i32) -> Vec3 {
    let (hit, dist) = intersect(spheres,r);
    match hit {
        None => Vec3::zero(), //we didn't intersect anything, return black
        Some(obj) => {
            //return Vec3::new(dist/(dist+1000.0));
            let x = r.origin+ (r.direction*dist);
            //return x.normalize();
            let un = (x - obj.position);
            let n = un.normalize();
            //check if we're inside the sphere or not
            let nl =  if n.dot(r.direction) < 0.0 {
                n
                //return Vec3::set(1.0,0.0,0.0);
            } else {
                //return Vec3::set(0.0,0.0,1.0);
                n * -1.0
            };
            //return Vec3::new(-nl.dot(r.direction));
            let mut f = obj.color;
            let p = if f.x>f.y && f.x>f.z {
                f.x
            } else if f.y > f.z {
                f.y
            } else {
                f.z
            };
            *depth +=1;
            if *depth > 5 { //begin russian roulette after 5 bounces
                            //and terminate at 25 bounces
                if *depth < 50 && rand::random::<f64>() < p {
                    f = f * (1.0/p);
                } else {
                    return obj.emission;
                }
            }
            /*if f.length() < 1e-6f64 {
                return obj.emission;
            }*/
            match obj.refl {
                ReflType::DIFF => {
                    let r1 = 2.0 * PI * rand::random::<f64>();
                    //let r2 = 0.5 * PI * ( 1.0 - f64::sqrt(rand::random::<f64>()));
                    let r2 = rand::random::<f64>();
                    let r2s = f64::sqrt(r2);//f64::sin(r2);//f64::sqrt(r2);
                    //let r2c = f64::cos(r2);
                    let w = nl;
                    let u = if f64::abs(w.x) > 0.1 {
                        Vec3::set(0.0,1.0,0.0)
                    } else {
                        Vec3::set(1.0,0.0,0.0)
                    }.cross(w).normalize();
                    let v = w.cross(u);
                    let d = (u*f64::cos(r1)*r2s + v*f64::sin(r1)*r2s + w*f64::sqrt(1.0-r2)).normalize();
                    //let d = (u*f64::cos(r1)*r2s + v*f64::sin(r1)*r2s + w*r2c).normalize();
                    obj.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                    //obj.emission + n.abs()
                },
                ReflType::SPEC => {
                    let d = r.direction - n*2.0*n.dot(r.direction);
                    obj.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                    //obj.emission + n.abs()
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
                        obj.emission + f * radiance(spheres,&reflRay,depth)
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
                        let tmp = if *depth > 2 {
                            if rand::random::<f64>() < P { //Russian roulette
                                radiance(spheres,&reflRay,depth)*RP
                            } else {
                                radiance(spheres,&Ray { origin: x, direction: tdir },depth)*TP
                            }
                        } else {
                            radiance(spheres,&reflRay,depth)*Re + radiance(spheres,&Ray { origin: x, direction: tdir }, depth)*Tr
                        };
                        obj.emission + f * tmp
                    }
                }
            }
        }
    }
}

fn main() {
    let spheres = Arc::new(vec![
        Sphere { //left wall
            radius: 1e5,
            position: Vec3::set(1e5f64+1.0,40.8,81.6),
            emission: Vec3::zero(),
            color: Vec3::set(0.75,0.25,0.25),
            refl: ReflType::DIFF,
        },
        Sphere { //right wall
            radius: 1e5,
            position: Vec3::set(-1e5f64+99f64,40.8,81.6),
            emission: Vec3::zero(),
            color: Vec3::set(0.25,0.25,0.75),
            refl: ReflType::DIFF,
        },
        Sphere { //back wall
            radius: 1e5,
            position: Vec3::set(50.0,40.8,1e5f64),
            emission: Vec3::zero(),
            color: Vec3::set(0.75,0.75,0.75),
            refl: ReflType::DIFF,
        },
        Sphere { //front wall
            radius: 1e5,
            position: Vec3::set(50.0,40.8,-1e5f64+170f64),
            emission: Vec3::zero(),
            color: Vec3::zero(),//Vec3::set(0.5,0.5,0.5),
            refl: ReflType::DIFF,
        },
        Sphere { //floor
            radius: 1e5,
            position: Vec3::set(50.0,1e5f64,81.6),
            emission: Vec3::zero(),
            color: Vec3::set(0.75,0.75,0.75),
            refl: ReflType::DIFF,
        },
        Sphere { //ceiling
            radius: 1e5,
            position: Vec3::set(50.0,-1e5f64+81.6,81.6),
            emission: Vec3::zero(),
            color: Vec3::new(0.75),//Vec3::set(0.75,0.75,0.75),
            refl: ReflType::DIFF,
        },
        Sphere { //mirror ball
            radius: 16.5,
            position: Vec3::set(27.0,16.5,47.0),
            emission: Vec3::zero(),
            color: Vec3::new(0.999),
            refl: ReflType::SPEC,
        },
        Sphere { //glass ball
            radius: 16.5,
            position: Vec3::set(73.0,16.5,78.0),
            emission: Vec3::zero(),
            color: Vec3::new(0.999),
            refl: ReflType::REFR,
        },
        Sphere { //light
            radius: 600.0,
            position: Vec3::set(50.0,681.6-0.27,81.6),
            emission: Vec3::new(12.0),
            color: Vec3::zero(),
            refl: ReflType::DIFF,
        },
    ]);

    let w = 1024;
    let h = 768;
    let samps = 25000/4;
    println!("generating image {}x{} with {} samples per pixel",w,h,samps*4);//4 subsamples per pixel for AA
    let mut c = vec![Vec3::new(0.0); w*h];
    let progress = Arc::new(AtomicUsize::new(0));
    unsafe { //Replace with scoped::ptr or equivalent someday
        //ideally we'd use scoped::ptr here but it's unstable so we create dc, which is c with unlimited lifetime
        //  this lets us create slices we can move into the threads without the compiler complaining about lifetimes
        //  we ensure safety by making sure all the threads join before releasing uc
        let uc: *mut Vec<Vec3> = &mut c as *mut Vec<Vec3>;
        let dc: &mut Vec<Vec3> = &mut *uc;
        let num_threads = 8;
        let section = h/num_threads;
        let chunk_length = w * section;
        let mut threads = Vec::with_capacity(num_threads);
        println!("will process {} lines per thread with {} threads total",section,num_threads);
        for (thread,mut chunk) in dc.chunks_mut(chunk_length).enumerate() {
            //create our per-thread variables that will be moved
            let cam = Ray { origin: Vec3::set(50.0,52.0,295.6), direction: Vec3::set(0.0,-0.042612,-1.0).normalize()};
            let cx = Vec3::set((w as f64)*0.5135/(h as f64),0.0,0.0);
            let cy = cx.cross(cam.direction).normalize()*0.5135;
            let my_spheres = spheres.clone();
            let thread_progress = progress.clone();
            let start = thread * section;
            let end = (thread + 1) * section - 1;
            threads.push(spawn(move || {
                    for i in 0..section {
                        let y = h - (i + start);
                        for x in 0..w {
                            let mut pixel = Vec3::zero();
                            for sy in 0..2 {
                                //let i = (h-y-1)*w+x;
                                for sx in 0..2 {
                                    let mut r = Vec3::zero();
                                    for s in 0..samps {
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
                                        r = r + radiance(&my_spheres, &Ray{origin:cam.origin+(d*140.0),direction:d.normalize()},&mut 0) * (1.0/(samps as f64));
                                    }
                                    pixel = pixel + (Vec3::set(clamp(r.x),clamp(r.y),clamp(r.z))*0.25);
                                }
                            }
                            chunk[i*w + x] = pixel;
                        }
                        thread_progress.fetch_add(1, Ordering::Relaxed);
                    }
                }));
        }
        let mut p = 0;
        print!("progress: 0.0%");
        while p < (h-1) {
            let np = progress.load(Ordering::Relaxed);
            if p != np {
                let frac = p as f64 / (h-1) as f64;
                print!("\rprogress: {:.1}%", frac * 100.0);
                std::io::stdout().flush();
                p = np;
            }
            sleep_ms(50);
        }
        for t in threads {
            t.join();
        }
        println!("\ndone!");
    }
    //single threaded version
    /*for y in 0..h {
        print!("\rRendering ({} spp) {}",samps*4,100.0*(y as f64)/(h as f64-1.0));
        for x in 0..w {
            let mut pixel = Vec3::zero();
            for sy in 0..2 {
                let i = (h-y-1)*w+x;
                for sx in 0..2 {
                    let mut r = Vec3::zero();
                    for s in 0..samps {
                        let r1 = 2.0*rand::random::<f64>();
                        let dx;
                        if r1 < 1.0 {
                            dx = f64::sqrt(r1)-1.0;
                        } else {
                            dx = 1.0-f64::sqrt(2.0-r1);
                        }
                        let r2 = 2.0*rand::random::<f64>();
                        let dy;
                        if r2 < 1.0 {
                            dy = f64::sqrt(r1)-1.0;
                        } else {
                            dy = 1.0-f64::sqrt(2.0-r1);
                        }
                        let d = cx * ( ( ((sx as f64)+0.5 + (dx as f64)) / 2.0 + (x as f64)) / (w as f64) - 0.5) +
                                cy * ( ( ((sy as f64)+0.5 + (dy as f64)) / 2.0 + (y as f64)) / (h as f64) - 0.5) + cam.direction;
                        r = r + radiance(&spheres, &Ray{origin:cam.origin+(d*140.0),direction:d},&mut 0) * (1.0/(samps as f64));
                    }
                    pixel = pixel + Vec3::set(clamp(r.x),clamp(r.y),clamp(r.z))*0.25;
                }
            }
            c[y*w + x] = pixel;
        }
    }*/
    print!("\rSaving...");
    let f = File::create("image.ppm").unwrap();
    let mut writer = BufWriter::new(f);
    write!(&mut writer,"P3\n{} {}\n{}\n",w,h,255).unwrap();
    for pixel in c {
        write!(&mut writer," {} {} {} ",toInt(pixel.x),toInt(pixel.y),toInt(pixel.z)).unwrap();
    }
}
