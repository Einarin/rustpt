extern crate rand;
extern crate crossbeam;
use std::fs::File;
use std::io::BufWriter;
use std::io::Write;
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

mod vec3;
use vec3::*;
mod primitives;
use primitives::*;
mod kdtree;
use kdtree::*;

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

fn load_mesh(path: &str) -> (Vec<Triangle>, KDTree) {
    let (obj, mat) = tobj::load_obj(&Path::new(path)).unwrap();
    let obj_tris = obj.into_iter().flat_map(|part| {
        let mut tris = Vec::new();
        for f in 0..part.mesh.indices.len() / 3 {
            let mut tri = Vec::new();
            for ind in &part.mesh.indices[3*f..3*f+3] {
                let x = part.mesh.positions[(3* *ind) as usize]as f64;
                let y = part.mesh.positions[(3* *ind) as usize +1]as f64;
                let z = part.mesh.positions[(3* *ind) as usize +2]as f64;
                let v = Vec3::set(-x,z,y);
                tri.push(v * 2.0 + Vec3::set(65.0,0.0,70.0));
            }
            tris.push(Triangle { points: [tri[0],tri[1],tri[2]]});
        }
        tris
    }).collect();
    let kdtree = KDTree::build(&obj_tris);
    (obj_tris, kdtree)
}

fn abs(val: f64) -> f64 {
    if val < 0.0 {
        val * -1.0
    } else {
        val
    }
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

        let x;
        let n;
        let mtl;
        if dist2 < dist {
            x = r.origin+ (r.direction*dist2);
            if let Some(triangle) = hit2 {
                //ReflType::DIFF
                let e1 = triangle.points[1] - triangle.points[0];
                let e2 = triangle.points[2] - triangle.points[0];
                let norm = e1.cross(e2);
                let r1 = 2.0 * PI * rand::random::<f64>();
                let r2 = rand::random::<f64>();
                let r2s = f64::sqrt(r2);
                n = norm.normalize();
                mtl = Material {
                    emission: Vec3::zero(),//Vec3::set(0.25,0.75,0.25) * abs(n.dot(x.normalize())),//Vec3::zero(),
                    color: Vec3::set(0.1,0.2,0.4),
                    refl: ReflType::DIFF,
                }
            } else {
                break;
            }
        } else {
            x = r.origin+ (r.direction*dist);
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
        if depth > 4 { //begin russian roulette after 5 bounces
                        //and terminate at 10 bounces
            if depth < 8 && rand::random::<f64>() < p * atten.length() {
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
            mesh_data: load_mesh("teapot.obj"),
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
                /*Sphere { //glass ball
                    radius: 16.5,
                    position: Vec3::set(73.0,16.5,78.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::new(0.999),
                        refl: ReflType::REFR,
                    },
                },*/
                Sphere { //gold hemisphere
                    radius: 16.5,
                    position: Vec3::set(27.0,0.0,96.0),
                    material: Material {
                        emission: Vec3::zero(),
                        color: Vec3::set(1.000, 0.766, 0.336),
                        refl: ReflType::SPEC,
                    },
                },
                /*Sphere { //green glass
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
                },*/
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
    let width = 640;//1024*2;
    let height = 480;//768*2;
    let scale = 1;
    let mut window: PistonWindow = WindowSettings::new(
            "rustpt",
            [width*scale,height*scale]
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
            draw_image(&texture, c.transform.scale(scale as f64,scale as f64), g);
            //draw_image(&texture, c.transform.scale(0.5,0.5), g);
            //draw_image(&texture, c.transform, g);
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
