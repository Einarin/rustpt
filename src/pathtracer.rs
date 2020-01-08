extern crate rand;
extern crate crossbeam;
extern crate rayon;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize,Ordering};
use std::time::{Instant};

use vec3::*;
use primitives::*;
use kdtree::*;
use unordered_access_buffer::UnorderedBufferWriter;


static PI: f64 = 3.14159265358979323;

pub struct PathTracer {
    pub spheres: Arc<Vec<Sphere>>,
    pub mesh_data: (Vec<MeshTriangle>, KDTree),
    pub materials: Vec<Material>,
    pub textures: Vec<PhysicalMaterialTextures>,
    pub rust_textures: Vec<Texture>,
    pub width: usize,
    pub height: usize,
    pub samples: u32,
    pub buffer: UnorderedBufferWriter<Vec3>//Vec<Vec3>,
}



impl PathTracer {
    #[allow(unused_variables)]
    pub fn new(width: u32, height: u32, back_buffer: UnorderedBufferWriter<Vec3>) -> PathTracer {
        //Vec3::set(65.0,0.0,70.0)
        //let (iron_tris,iron_materials) = load_mesh("IronMan.obj");
        //let (nano_tris,nano_materials) = load_mesh("nanosuit.obj");
        let (teapot_tris,teapot_materials) = load_mesh("teapot.obj");
        //let (cube_tris,cube_materials) = load_mesh("cube.obj");
        //let (fancy_tris,fancy_materials) = load_mesh("fancycube.obj");
        /*println!("materials: {:?}", fancy_materials);
        let textures: Vec<PhysicalMaterialTextures> = fancy_materials.iter().map(|material|{
            let path = &material.diffuse_texture;
            let albedo = if path.is_empty() {
                None
            } else {
                Texture::from_path(path).ok()
            };
            let path = &material.normal_texture; //for some reason Blender exports roughness into the normal map slot in the mtl file
            let roughness = if path.is_empty() {
                None
            } else {
                Texture::from_path(path).ok()
            };
            let metalness = if let Some(path) = material.unknown_param.get("refl") {
                if path.is_empty() {
                    None
                } else {
                    Texture::from_path(path).ok()
                }
            } else {
                None
            };
            PhysicalMaterialTextures {
                albedo,
                roughness,
                metalness
            }
        }).collect();
        let rust_textures: Vec<Texture> = ["rustediron2-Unreal-Engine/rustediron2_basecolor.png","rustediron2-Unreal-Engine/rustediron2_metallic.png","rustediron2-Unreal-Engine/rustediron2_roughness.png"].iter().map(|path|{
            Texture::from_path(path).unwrap()
        }).collect();
        println!("textures: {:?}", textures);
        let mut materials: Vec<Material> = fancy_materials.iter().map(|material|{
            //attempt to translate obj lighting model into physically based data
            let roughness = (900.0f64 - material.shininess as f64) / 1000.0; //translate shinyness to roughness
            match material.illumination_model {
                Some(0...1) => Material::Diffuse(Vec3::from_f32_slice(&material.diffuse)),
                Some(2) => Material::Physical(PhysicalMaterial {
                    albedo: Vec3::from_f32_slice(&material.diffuse),
                    metalness: 0.0,
                    roughness,
                }),
                Some(3) => Material::Physical(PhysicalMaterial {
                    albedo: Vec3::from_f32_slice(&material.diffuse),
                    metalness: 1.0,
                    roughness,
                }),
                //Some(4...9) => Material::Refractive(Vec3::from_f32_slice(&material.diffuse)),
                _ =>  Material::Physical(PhysicalMaterial {
                    albedo: Vec3::from_f32_slice(&material.diffuse),
                    metalness: 0.0,
                    roughness,
                })
            }
        }).collect();*/
        
        
        /*let materials = vec![
             Material {
                    emission: Vec3::zero(),//Vec3::set(0.25,0.75,0.25) * abs(n.dot(x.normalize())),//Vec3::zero(),
                    color: Vec3::new(0.9),//Vec3::set(0.1,0.2,0.4),
                    refl: ReflType::REFR,
            },
            Material {
                    emission: Vec3::zero(),//Vec3::set(0.25,0.75,0.25) * abs(n.dot(x.normalize())),//Vec3::zero(),
                    color: Vec3::new(0.7),//Vec3::set(0.1,0.2,0.4),
                    refl: ReflType::SPEC,
            },
            Material {
                    emission: Vec3::zero(),//Vec3::set(0.25,0.75,0.25) * abs(n.dot(x.normalize())),//Vec3::zero(),
                    color: Vec3::new(0.1),//Vec3::set(0.1,0.2,0.4),
                    refl: ReflType::DIFF,
            }
        ];*/
        let mut triangles: Vec<MeshTriangle> = Vec::new();
        //triangles.extend(cube_tris.iter().map(|tri|{let mut transformed = *tri * Vec3::new(10.0) + Vec3::set(60.0, 30.0, 50.0); transformed.material_index = 0; transformed}));

        //triangles.extend(nano_tris.iter().map(|tri|*tri * Vec3::new(2.0) + Vec3::set(50.0,0.0,50.0)));
        //triangles.extend(iron_tris.iter().map(|tri|*tri * Vec3::new(0.1) + Vec3::set(50.0,0.0,90.0)));
        //triangles.extend(fancy_tris.iter().map(|tri|*tri * Vec3::new(4.0) + Vec3::set(20.0,15.0,40.0)));
        //materials.push(Material::Diffuse(Vec3::new(0.1)));
        /*triangles.extend(nano_tris.iter().map(|tri|{let mut transformed = *tri * Vec3::new(2.0) + Vec3::set(50.0,0.0,50.0); transformed.material_index = materials.len()-1; transformed}));
        materials.push(Material {
            emission: Vec3::zero(),
            color: Vec3::new(0.999),
            refl: ReflType::PBR_REFR,
        });
        triangles.extend(nano_tris.iter().map(|tri|{let mut transformed = *tri * Vec3::new(2.0) + Vec3::set(70.0,0.0,70.0); transformed.material_index = materials.len()-1; transformed}));
        materials.push(Material {
            emission: Vec3::zero(),
            color: Vec3::new(0.7),
            refl: ReflType::PBR,
        });*/
        //triangles.extend(nano_tris.iter().map(|tri|{let mut transformed = *tri * Vec3::new(2.0) + Vec3::set(30.0,0.0,70.0); transformed.material_index = materials.len()-1; transformed}));
        let mut materials = Vec::new();
        let textures = Vec::new();
        let rust_textures = Vec::new();
        materials.push(Material::Refractive(Vec3::new(0.999)));
        triangles.extend(teapot_tris.iter().map(|tri|{
            let mut transformed = *tri;
            for i in 0..3 {
                let mut pt = &mut transformed.pos.points[i];
                pt.x = -pt.x;
                let tmp = pt.y;
                pt.y = pt.z;
                pt.z = tmp;
            }
            transformed.material_index = materials.len()-1;
            transformed * Vec3::new(0.75) + Vec3::set(75.0,0.0,100.0)
        }));

        println!("Building KDTree with {} triangles...",triangles.len());
        let kdtree = KDTree::build(&triangles);
        println!("Done building KDTree.");
        //kdtree.verify(&triangles);
        PathTracer {
            width: width as usize,
            height: height as usize,
            samples: 0,
            buffer: back_buffer,//vec![Vec3::new(0.0);(width*height) as usize],
            mesh_data: (triangles,kdtree),
            materials,
            textures,
            rust_textures,
            spheres: Arc::new(vec![
                Sphere { //left wall
                    radius: 1e5,
                    position: Vec3::set(1e5f64+1.0,40.8,81.6),
                    material: Material::Specular(Vec3::set(0.75,0.25,0.25)),
                },
                Sphere { //right wall
                    radius: 1e5,
                    position: Vec3::set(-1e5f64+99f64,40.8,81.6),
                    material: Material::Diffuse(Vec3::set(0.25,0.25,0.75)),
                },
                Sphere { //back wall
                    radius: 1e5,
                    position: Vec3::set(50.0,40.8,1e5f64),
                    material: Material::Diffuse(Vec3::set(0.75,0.75,0.75)),
                },
                Sphere { //front wall
                    radius: 1e5,
                    position: Vec3::set(50.0,40.8,-1e5f64+170f64),
                    material: Material::Diffuse(Vec3::zero()),//Vec3::set(0.5,0.5,0.5),
                },
                Sphere { //floor
                    radius: 1e5,
                    position: Vec3::set(50.0,1e5f64,81.6),
                    material: Material::Diffuse(Vec3::set(0.75,0.75,0.75)),
                },
                Sphere { //ceiling
                    radius: 1e5,
                    position: Vec3::set(50.0,-1e5f64+81.6,81.6),
                    material: Material::Diffuse(Vec3::new(0.75)),
                },
                /*Sphere { //mirror ball
                    radius: 16.5,
                    position: Vec3::set(27.0,16.5,47.0),
                    material: Material::Diffuse(Vec3::new(0.999)),
                },*/
                Sphere { //glass ball
                    radius: 12.0,
                    position:  Vec3::set(12.0,12.0,90.0),
                    material: Material::Refractive(Vec3::set(0.99,0.99,0.99)),
                },
                /*Sphere { //blue plastic ball
                    radius: 16.5,
                    position: Vec3::set(27.0,16.5,78.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(0.1, 0.3, 0.9),
                        metalness: 0.0,
                        roughness: 0.1
                    }),
                },*/
                /*Sphere { //gold hemisphere
                    radius: 16.5,
                    position: Vec3::set(27.0,0.0,96.0),
                    //material: Material::Specular(Vec3::set(1.000, 0.766, 0.336)),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.2
                    })
                },*/
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(15.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.0
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(25.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.1
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(35.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.2
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(45.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.3
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(55.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.4
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(65.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.5
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(75.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.6
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(85.0,50.0,40.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 1.0,
                        roughness: 0.7
                    })
                },
////////////////////////////////////////////////////////////////////
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(15.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.0
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(25.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.1
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(35.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.2
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(45.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.3
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(55.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.4
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(65.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.5
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(75.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.6
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(85.0,60.0,30.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.5,
                        roughness: 0.7
                    })
                },
////////////////////////////////////////////
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(15.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.0
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(25.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.1
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(35.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.2
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(45.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.3
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(55.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.4
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(65.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.5
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(75.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.6
                    })
                },
                Sphere { //gold hemisphere
                    radius: 5.0,
                    position: Vec3::set(85.0,70.0,20.0),
                    material: Material::Physical(PhysicalMaterial{
                        albedo: Vec3::set(1.000, 0.766, 0.336),
                        metalness: 0.0,
                        roughness: 0.7
                    })
                },


                /*Sphere { //green glass
                    radius: 8.0,
                    position: Vec3::set(50.0,8.0,110.0),
                    material: Material {
                        emission: Vec3::set(0.0,0.0,0.0),
                        color: Vec3::set(0.56,0.99,0.56),
                        refl: ReflType::PBR_REFR,
                    },
                },*/
                /*Sphere { //purple glass
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
                    material: Material::Light(Vec3::new(12.0)),
                },
            ]),
        }
    }

    pub fn render(&mut self, samples_per_pass: u32, sample_count: &Arc<AtomicUsize>) {
        let ref spheres = self.spheres;
        let (ref tris,ref kdtree) = self.mesh_data;
        //kdtree.verify(tris);
        let mesh_data = (tris,kdtree);
        let materials = &self.materials;
        let textures = &self.textures;
        let w = self.width;
        let h = self.height;
        //println!("generating image {}x{} with {} samples per pixel",w,h,samps*4);//4 subsamples per pixel for AA
        let ref mut c = self.buffer;
        let progress = Arc::new(AtomicUsize::new(0));
        let start = Instant::now();
        rayon::scope(|scope| {
            let num_threads = h;
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
                let mut scratch_space = Vec::with_capacity(kdtree.intersect_stack_size);
                threads.push(scope.spawn(move |_| {
                        for i in 0..section {
                            let y = h - (i + start);
                            for x in 0..w {
                                let mut pixel = Vec3::zero();
                                /*if let Some(ref tex) = diffuse_textures[0] {
                                    let px = tex.sample(x as f32 / w as f32, y as f32 / h as f32);
                                    //let px = tex.index(x,y);
                                    pixel = Vec3::set(px[0] as f64, px[1] as f64, px[2] as f64);
                                }*/
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
                                            let d = cx * ( ( ((sx as u32 as f64)+0.5 + dx ) / 2.0 + (x as f64)) / (w as f64) - 0.5) +
                                                    cy * ( ( ((sy as u32 as f64)+0.5 + dy ) / 2.0 + (y as f64)) / (h as f64) - 0.5) + cam.direction;
                                            //r = r + radiance(&my_spheres, &Ray{origin:cam.origin+(d*140.0),direction:d.normalize()},&mut 0) * (1.0/(samps as f64));
                                            r = r + radiance_iter(&my_spheres, mesh_data, materials, textures, Ray{origin:cam.origin+(d*140.0),direction:d.normalize()},&mut scratch_space);
                                            /*if let Some(ref tex) = diffuse_textures[0] {
                                                let px = tex.sample(x as f32 / w as f32, y as f32 / h as f32);
                                                r = Vec3::set(px[0] as f64, px[1] as f64, px[2] as f64);
                                            }*/
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
            /*let mut p = 0;
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
            }*/
            /*for t in threads {
                t.join();
            }*/
            //self.samples += samples_per_pass;
            sample_count.fetch_add(samples_per_pass as usize, Ordering::Relaxed);
            //println!("\ndone!");
        });
        let duration = start.elapsed();
        let seconds = duration.as_secs() as f64 + (duration.subsec_nanos() as f64)/1e9f64;
        println!("Time: {} seconds",seconds);
    }
}

#[allow(non_snake_case)] //allow variable naming here to match smallpt
fn radiance_iter(spheres: &Vec<Sphere>, mesh_data: (&Vec<MeshTriangle>, &KDTree), materials: &[Material], textures: &[PhysicalMaterialTextures], ray: Ray, scratch_space: &mut Vec<usize>) -> Vec3 {
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
        depth += 1;
        if depth > 4 { //begin russian roulette
            let p = 0.5;
            if /*atten.x.max(atten.y.max(atten.z)) > 0.001 &&*/ depth < 8 && rand::random::<f64>() < p * atten.length() {
                atten = atten * (1.0/p);
            } else {
                break;
            }
        }
        let (hit, dist) = intersect_val(spheres,r);
        //let (hit2, dist2, coords) = intersect_triangle(obj_tris, r);
        //let (hit2, dist2): (Option<Triangle>,f64) = (None,1e20f64);
        let (hit2, dist2, coords) = kdtree.intersect2(&obj_tris, r, scratch_space);
        //let hit2: Option<Triangle> = None; let dist2 = std::f64::INFINITY;
        /*if hit2 != hit3 || dist2 != dist3 {
            println!("Ray {:?} didn't intersect {:?} like it should have! Got {:?} instead - {:?} {:?}",r,hit3, hit2, dist3, dist2);
        }*/
        let x;
        let n;
        let mut mtl;
        if dist2 < dist {
            x = r.origin+ (r.direction*dist2);
            if let Some(triangle) = hit2 {
                //ReflType::DIFF
                let e1 = triangle.pos.points[1] - triangle.pos.points[0];
                let e2 = triangle.pos.points[2] - triangle.pos.points[0];
                let norm = e1.cross(e2);
                //let r1 = 2.0 * PI * rand::random::<f64>();
                //let r2 = rand::random::<f64>();
                //let r2s = f64::sqrt(r2);
                n = norm.normalize();
                mtl = materials[triangle.material_index];/*Material {
                    emission: Vec3::zero(),//Vec3::set(0.25,0.75,0.25) * abs(n.dot(x.normalize())),//Vec3::zero(),
                    color: Vec3::new(0.9),//Vec3::set(0.1,0.2,0.4),
                    refl: ReflType::REFR,
                }*/
                /*mtl = Material {
                    emission: Vec3::zero(),
                    color: Vec3::set(1.000, 0.766, 0.336),
                    refl: ReflType::SPEC,
                }*/
                if triangle.material_index < textures.len() {
                    let texture_set = &textures[triangle.material_index];
                    /*if let Some(ref texture) = diffuse_textures[triangle.material_index] {
                        //mtl.refl = ReflType::DebugColor;
                        let mut texcoord = [0.0f32; 2];
                        for i in 0..3 {
                            texcoord[0] += coords[i] as f32 * triangle.texcoord[i][0];
                            texcoord[1] += coords[i] as f32 * triangle.texcoord[i][1];
                        }
                        let sample = texture.sample(texcoord[0], texcoord[1]);
                        /*if (sample[0] as u32) + (sample[1] as u32) + (sample[2] as u32) > 0 {
                            mtl.refl = ReflType::DebugColor;
                        }*/
                        mtl = Material::Diffuse(Vec3::set( sample[0] as f64 / 255.0, sample[1] as f64 / 255.0, sample[2] as f64 / 255.0));
                        //mtl.emission = Vec3::zero();
                        //print!("{:?}",mtl.color);
                        //mtl = Material::DebugColor(Vec3::set(texcoord[0] as f64, texcoord[1] as f64, 0.0));
                        //mtl.color = Vec3::set(texcoord[0] as f64, texcoord[1] as f64, 0.0);
                        //mtl.color = Vec3::from_iter(&mut sample.into_iter().map(|val| *val as f64 / std::u8::MAX as f64));
                    } else {
                        //mtl = Material::Diffuse(Vec3::set(1.0,0.0,1.0));
                    }*/
                    if let Some(ref albedo_tex) = texture_set.albedo {
                        if let Material::Physical(ref mut props) = mtl {
                            let mut texcoord = [0.0f32; 2];
                            for i in 0..3 {
                                texcoord[0] += coords[i] as f32 * triangle.texcoord[i][0];
                                texcoord[1] += coords[i] as f32 * triangle.texcoord[i][1];
                            }
                            let sample = albedo_tex.sample(texcoord[0], texcoord[1]);                                                                                                      
                            props.albedo = Vec3::set( sample[0] as f64 / 255.0, sample[1] as f64 / 255.0, sample[2] as f64 / 255.0);
                        }
                    }
                    if let Some(ref metalness_tex) = texture_set.metalness {
                        if let Material::Physical(ref mut props) = mtl {
                            let mut texcoord = [0.0f32; 2];
                            for i in 0..3 {
                                texcoord[0] += coords[i] as f32 * triangle.texcoord[i][0];
                                texcoord[1] += coords[i] as f32 * triangle.texcoord[i][1];
                            }
                            let sample = metalness_tex.sample(texcoord[0], texcoord[1]);                                                                                                      
                            props.metalness = Vec3::set( sample[0] as f64 / 255.0, sample[1] as f64 / 255.0, sample[2] as f64 / 255.0).length();
                        }
                    }
                    if let Some(ref roughness_tex) = texture_set.roughness {
                        if let Material::Physical(ref mut props) = mtl {
                            let mut texcoord = [0.0f32; 2];
                            for i in 0..3 {
                                texcoord[0] += coords[i] as f32 * triangle.texcoord[i][0];
                                texcoord[1] += coords[i] as f32 * triangle.texcoord[i][1];
                            }
                            let sample = roughness_tex.sample(texcoord[0], texcoord[1]);                                                                                                      
                            props.roughness = Vec3::set( sample[0] as f64 / 255.0, sample[1] as f64 / 255.0, sample[2] as f64 / 255.0).length();
                        }
                    }
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
        /*let mut f = mtl.color;
        let p = if f.x>f.y && f.x>f.z {
            f.x
        } else if f.y > f.z {
            f.y
        } else {
            f.z
        };*/
        
        match mtl {
            Material::Light(emission) => {
                radiance = radiance + (atten * emission);
                break;
            },
            Material::Diffuse(color) => {
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
                //radiance = radiance + (atten * mtl.emission);
                atten = atten * color;
                r = Ray{origin:x,direction:d};
            },
            Material::Specular(color) => {
                let d = r.direction - n*2.0*n.dot(r.direction);
                //mtl.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                //radiance = radiance + (atten * mtl.emission);
                atten = atten * color;
                r = Ray{origin:x,direction:d};
            },
            Material::Refractive(color) => {
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
                //radiance = radiance + (atten * mtl.emission);
                if cos2t < 0.0 { //total internal reflection
                    //mtl.emission + f * radiance(spheres,&reflRay,depth)
                    atten = atten * color;
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
                        atten = atten * TP * color;
                        r = Ray { origin: x, direction: tdir };
                    }
                }
            },
            Material::DebugNormal => {
                radiance = radiance + atten * n.abs();
                break;
            },
            Material::DebugColor(color) => {
                radiance = radiance + atten * color;
                break;
            },
            Material::Physical(mtl) => {
                let albedo = mtl.albedo;
                let metalness = mtl.metalness;
                let roughness = mtl.roughness;
                let diffuse = albedo * (1.0 - metalness);
                let specular = mix(Vec3::new(1.0),albedo,metalness);
                //let metalness = metalness.max(0.04);
                let spec_d = r.direction - n*2.0*n.dot(r.direction);

                let incident = 1.0 - nl.dot(r.direction);
                let fresnel = incident.powf(5.0);// * 1.0f64.powf(10000.0f64.powf(roughness));
                //let metalness = metalness + (1.0-metalness)*fresnel.max(0.0);

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
                let diff_d = (u*f64::cos(r1)*r2s + v*f64::sin(r1)*r2s + w*f64::sqrt(1.0-r2)).normalize();

                let d;
                let color;
                let specular_probability = metalness.max(0.04);
                if specular_probability < rand::random::<f64>() {
                    d = diff_d;
                    color = diffuse;
                } else {
                    d = mix(spec_d,diff_d,roughness);
                    color = specular;
                }
                //let d = mix(spec_d,diff_d,roughness);
                //let color = mix(diffuse,specular,metalness);

                //mtl.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                //radiance = radiance + (atten * mtl.emission);
                atten = atten * color;
                r = Ray{origin:x,direction:d};
            },
            /*PBR_REFR => {
                let metalness = 1.0;
                let roughness = 0.2;
                let diffuse = mtl.color;// * (1.0 - metalness);
                let specular = mix(Vec3::new(0.04),mtl.color,metalness);
                
                let spec_d = r.direction - n*2.0*n.dot(r.direction);

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
                let diff_d = (u*f64::cos(r1)*r2s + v*f64::sin(r1)*r2s + w*f64::sqrt(1.0-r2)).normalize();

                let d = mix(spec_d,diff_d,roughness);
                let color = mix(specular,diffuse,roughness);

                //mtl.emission + f * radiance(spheres,&Ray{origin:x,direction:d},depth)
                radiance = radiance + (atten * mtl.emission);
                atten = atten * color;
                let reflRay = Ray{origin:x,direction:d};

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
                    if rand::random::<f64>() < P {
                        atten = atten * RP;
                        r = reflRay;
                    } else {
                        atten = atten * TP * mtl.color;
                        r = Ray { origin: x, direction: mix(tdir,sign * diff_d,roughness).normalize() };
                    }
                }
            }*/
        }
    }
    radiance
}

/*#[allow(non_snake_case)] //allow variable naming here to match smallpt
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
                },
                ReflType::DebugNormal => {
                    n.abs()
                }
                ReflType::DebugColor => {
                    obj.material.color
                }
            }
        }
    }
}*/