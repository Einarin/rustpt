
use std::thread::spawn;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize,Ordering};


use std::path::Path;
use std::f64;
use std::time::{Duration,Instant};

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
mod pathtracer;
use pathtracer::*;
mod UnorderedAccessBuffer;
//use UnorderedAccessBuffer::*;

fn clamp(x: f64) -> f64 {
    match x {
        _ if x < 0.0 => 0.0,
        _ if x > 1.0 => 1.0,
        _ => x
    }
}

fn to_u8(x: f64) -> u8 {
    (f64::powf(clamp(x),1.0/2.2)*255.0 + 0.5) as u8
}

struct Array4Iterable([u8; 4]);

struct Array4Iterator {
    array: [u8; 4],
    index: usize,
}

impl Iterator for Array4Iterator {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        let i = self.index;
        self.index += 1;
        if i < 4 {
            Some(self.array[i])
        } else {
            None
        }
    }
}

impl IntoIterator for Array4Iterable {
    type Item = u8;
    type IntoIter = Array4Iterator;
    fn into_iter(self) -> Self::IntoIter {
        Array4Iterator{
            array: self.0,
            index: 0,
        }
    }
}

pub fn main() {
    let opengl = OpenGL::V3_2;
    let width = 1024;//1024*2;
    let height = 768;//768*2;
    let scale = 1;
    let mut window: PistonWindow = WindowSettings::new(
            "rustpt",
            [width*scale,height*scale]
        )
        .opengl(opengl)
        .exit_on_esc(true)
        .build().unwrap();
    let (mut writer, reader) = UnorderedAccessBuffer::new((width * height) as usize);
    let mut samples_per_pass = 1;
    println!("Rendering...");
    let sample_count = Arc::new(AtomicUsize::new(0));
    let render_sample_count = sample_count.clone();
    spawn(move ||{
        let mut tracer = PathTracer::new(width,height, writer);
        loop{
            tracer.render(samples_per_pass,&render_sample_count);
        }
    });
    //tracer.render(samples_per_pass);
    let tone_map = |pixel: &Vec3|{let p = *pixel / (sample_count.load(Ordering::Relaxed) as f64); Array4Iterable([to_u8(p.x),to_u8(p.y),to_u8(p.z),255])};
    let buffer = reader.iter().flat_map(|x|tone_map(x)).collect();
    let img = image::ImageBuffer::from_raw(width,height,buffer).unwrap();
    let mut texture = Texture::from_image(&mut window.factory,&img,&TextureSettings::new()).unwrap();
    //let mut count = 1;
    let mut since_save = Instant::now();
    while let Some(e) = window.next() {        
        window.draw_2d(&e, |c, g| {
            //print!("Pass {} with {} samples",count,samples_per_pass);
            //tracer.render(samples_per_pass);
            //count += 1;
            samples_per_pass = std::cmp::min(64,samples_per_pass * 2);
            clear([0.5, 0.0, 1.0, 1.0], g);
            let buffer = reader.iter().flat_map(|x|tone_map(x)).collect();
            let img = image::ImageBuffer::from_raw(width,height,buffer).unwrap();
            texture.update(&mut g.encoder,&img).unwrap();
            draw_image(&texture, c.transform.scale(scale as f64,scale as f64), g);
            //draw_image(&texture, c.transform.scale(0.5,0.5), g);
            //draw_image(&texture, c.transform, g);
            //autosave once a minute
            if since_save.elapsed().as_secs() > 60 {
                let rgb_buffer: Vec<u8> = reader.iter().flat_map(|pixel|tone_map(pixel)).collect();
                spawn(move || {
                    image::save_buffer(&Path::new("save.png"), &rgb_buffer, width, height, image::RGBA(8)).unwrap();
                    println!("...Saved");
                });
                since_save = Instant::now();
            }
        });
    }
    println!("Closing");
    
    /*let f = File::create("image.ppm").unwrap();
    let mut writer = BufWriter::new(f);
    write!(&mut writer,"P3\n{} {}\n{}\n",width,height,255).unwrap();
    for pixel in tracer.buffer {
        write!(&mut writer," {} {} {} ",to_int(pixel.x),to_int(pixel.y),to_int(pixel.z)).unwrap();
    }*/
    let rgb_buffer: Vec<u8> = reader.iter().flat_map(|x|tone_map(x)).collect();
    image::save_buffer(&Path::new("image.png"), &rgb_buffer, width, height, image::RGBA(8)).unwrap();
}
