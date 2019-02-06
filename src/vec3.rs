use std::ops::*;

#[derive(Copy,Clone,PartialEq,PartialOrd,Debug)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn zero() -> Vec3 {
        Vec3 { x:0.0, y:0.0, z:0.0 }
    }
    pub fn new(all: f64) -> Vec3 {
        Vec3 { x: all, y: all, z: all }
    }
    pub fn set(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x: x, y: y, z: z }
    }
    pub fn length(self) -> f64 {
        let x2 = self.x*self.x;
        let y2 = self.y*self.y;
        let z2 = self.z*self.z;
        f64::sqrt( x2 + y2 + z2)
    }
    pub fn normalize(mut self) -> Vec3 {

        let denom = self.length();
        self = self * (1.0/denom);
        self
    }

    pub fn dot(self, rhs: Vec3) -> f64 {
        (self.x * rhs.x) + (self.y * rhs.y) + (self.z * rhs.z)
    }

    pub fn cross(self, rhs: Vec3) -> Vec3 {
        Vec3{ x: self.y*rhs.z-self.z*rhs.y, y: self.z*rhs.x-self.x*rhs.z, z: self.x*rhs.y-self.y*rhs.x }
    }

    pub fn abs(self) -> Vec3 {
        Vec3{ x: f64::abs(self.x), y: f64::abs(self.y), z: f64::abs(self.z)}
    }
}

impl Add for Vec3 {
    type Output = Vec3;

    fn add(self, _rhs: Vec3) -> Vec3 {
        Vec3 { x: self.x + _rhs.x, y:self.y + _rhs.y, z: self.z + _rhs.z }
    }
}

impl Add<f64> for Vec3 {
    type Output = Vec3;
    fn add(self, _rhs: f64) -> Vec3 {
        Vec3 { x:self.x + _rhs, y:self.y + _rhs, z:self.z + _rhs }
    }
}

impl Sub for Vec3 {
    type Output = Vec3;

    fn sub(self, _rhs: Vec3) -> Vec3 {
        Vec3 { x: self.x - _rhs.x, y: self.y - _rhs.y, z: self.z - _rhs.z }
    }
}

impl Mul<Vec3> for Vec3 {
    type Output = Vec3;
    fn mul(self, _rhs: Vec3) -> Vec3 {
        Vec3 { x: self.x * _rhs.x, y: self.y * _rhs.y, z: self.z * _rhs.z }
    }
}

impl Mul<f64> for Vec3 {
    type Output = Vec3;
    fn mul(self, _rhs: f64) -> Vec3 {
        Vec3 { x: self.x * _rhs, y: self.y * _rhs, z: self.z * _rhs }
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;
    fn mul(self, _rhs: Vec3) -> Vec3 {
        Vec3 { x: self * _rhs.x, y: self * _rhs.y, z: self * _rhs.z }
    }
}

impl Div<Vec3> for Vec3 {
    type Output = Vec3;
    fn div(self, _rhs: Vec3) -> Vec3 {
        Vec3{ x: self.x / _rhs.x, y: self.y / _rhs.y, z: self.z / _rhs.z }
    }
}

impl Div<f64> for Vec3 {
    type Output = Vec3;
    fn div(self, _rhs: f64) -> Vec3 {
        Vec3{ x: self.x / _rhs, y: self.y / _rhs, z: self.z / _rhs }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn vec_norm() {
        let vec = Vec3::set(2.0,2.0,1.0);
        println!("vec: {:?}",vec);
        let norm = vec.normalize();
        println!("norm: {:?}",norm);
        assert_eq!(norm.x,2.0/3.0);
        assert_eq!(norm.y,2.0/3.0);
        assert_eq!(norm.z,1.0/3.0);
    }

    #[test]
    fn vec_dot() {
        let xdir = Vec3::set(2.0,0.0,0.0);
        let ydir = Vec3::set(0.0,1.0,0.0);
        assert_eq!(xdir.dot(ydir),0.0);
        println!("xdir: {:?}",xdir);
        assert_eq!(xdir.dot(xdir), 4.0);
    }

    #[test]
    fn vec_cross() {
        let xdir = Vec3::set(1.0,0.0,0.0);
        let ydir = Vec3::set(0.0,1.0,0.0);
        let zdir = Vec3::set(0.0,0.0,1.0);
        assert_eq!(xdir.cross(ydir),zdir);
    }

    #[test]
    fn vec3_add() {
        let ones = Vec3::new(1.0f64);
        let twos = Vec3::new(2.0f64);
        let threes = ones + twos;
        assert_eq!(threes.x, 3.0f64);
        assert_eq!(threes.y, 3.0f64);
        assert_eq!(threes.z, 3.0f64);
    }

    #[test]
    fn vec3_sub() {
        let ones = Vec3::new(1.0f64);
        let twos = Vec3::new(2.0f64);
        let result = twos - ones;
        assert_eq!(result.x, 1.0f64);
        assert_eq!(result.y, 1.0f64);
        assert_eq!(result.z, 1.0f64);
    }

    #[test]
    fn vec3_mul() {
        let twos = Vec3::new(2.0f64);
        let result = twos * twos;
        assert_eq!(result.x, 4.0f64);
        assert_eq!(result.y, 4.0f64);
        assert_eq!(result.z, 4.0f64);
        let result = twos * 2.0;
        assert_eq!(result.x, 4.0f64);
        assert_eq!(result.y, 4.0f64);
        assert_eq!(result.z, 4.0f64);
    }

    #[test]
    fn vec3_abs() {
        let neg = Vec3::new(-1.0f64);
        let result = neg.abs();
        assert_eq!(result.x,1.0f64);
        assert_eq!(result.y,1.0f64);
        assert_eq!(result.z,1.0f64);
    }
}
