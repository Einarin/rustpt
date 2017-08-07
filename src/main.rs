
use std::thread::spawn;


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
mod pathtracer;
use pathtracer::*;

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
    let buffer = tracer.buffer.iter().flat_map(|pixel|{let p = *pixel * (1.0/(tracer.samples as f64)); vec![to_u8(p.x),to_u8(p.y),to_u8(p.z),255]}).collect();
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
