use std;
use std::f64;
use std::path::Path;
use std::ops::IndexMut;
use std::cmp::max;

use tobj;


use vec3::*;
use primitives::*;

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
    pub fn contains_point(&self, point: &Vec3) -> bool {
        point.x > self.x.0 && point.x < self.x.1
            && point.y > self.y.0 && point.y < self.y.1
            && point.z > self.z.0 && point.z < self.z.1
    }
    pub fn contains_ray(&self, ray: Ray) -> bool {
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
        //The naive check of the corners is _not_ correct
        //self.contains_point(&tri.points[0]) || self.contains_point(&tri.points[1]) || self.contains_point(&tri.points[2])
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
    fn intersects(&self, other: &AABB) -> bool {
        f64::max(self.x.0,other.x.0) < f64::min(self.x.1,other.x.1)
        && f64::max(self.y.0,other.y.0) < f64::min(self.y.1,other.y.1)
        && f64::max(self.z.0,other.z.0) < f64::min(self.z.1,other.z.1)
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
    pub fn from_tri(triangle : &Triangle) -> AABB {
        let mut mins = Vec3::new(f64::INFINITY);
        let mut maxs = Vec3::new(f64::NEG_INFINITY);
        for pt in triangle.iter() {
            mins.x = f64::min(mins.x, pt.x);
            mins.y = f64::min(mins.y, pt.y);
            mins.z = f64::min(mins.y, pt.z);
            maxs.x = f64::max(maxs.x, pt.x);
            maxs.y = f64::max(maxs.y, pt.y);
            maxs.z = f64::max(maxs.y, pt.z);
        }
        AABB {
            x: (mins.x,maxs.x),
            y: (mins.y,maxs.y),
            z: (mins.z,maxs.z)
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Axis {
    X,
    Y,
    Z,
}

#[derive(Debug)]
struct KDBranch {
    aabb: AABB,
    split_axis: Axis,
    split_val: f64,
    child_lt: usize,//Box<KDNode>,
    child_gt: usize,//Box<KDNode>,
}
#[derive(Debug)]
struct KDLeaf {
    tri_inds: Vec<usize>,
}
#[derive(Debug)]
enum KDNode {
    Branch(KDBranch),
    Leaf(KDLeaf),
}
#[derive(Debug)]
pub struct KDTree {
    root: Vec<KDNode>,
    pub intersect_stack_size: usize,
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
    /**
     * returns a tuple where the first element is the index of the root of this subtree and the second is the maximum depth encountered
     */
    fn build_helper(triangles: &Vec<Triangle>, aabb:AABB, depth: usize, node_vec: &mut Vec<KDNode>) -> (usize,usize) {
        let mut sum = Vec3::zero();
        let mut count:u32 = 0;
        let mut indices = Vec::new();
        for i in 0..triangles.len() {
            let ref tri = triangles[i];
            if /*aabb.contains_tri(tri)*/ aabb.intersects(&AABB::from_tri(&tri)) {
                indices.push(i);
                for pt in tri.iter() {
                    sum = sum + *pt;
                    count += 1;
                }
            }
        }
        let (biggest_axis, sval) = aabb.biggest_dim(sum * (1.0/(count as f64)));
        let (la, ga) = aabb.split_biggest(sum * (1.0/(count as f64)));
        //println!("count:{} sval:{} axis:{:?} la:{:?} ga:{:?}",count,sval,biggest_axis,la,ga);
        if depth < 10 && count > 6 && la.valid() && ga.valid() { //arbitrarily picked 20 triangles as leaf size
            let idx = node_vec.len();
            //println!("branch at {}",idx);
            node_vec.push(KDNode::Branch(KDBranch{
                aabb: aabb,
                split_axis: biggest_axis,
                split_val: sval,
                child_lt: 0,
                child_gt: 0,
                }));
            let (lt,ld) = KDTree::build_helper(triangles,la,depth+1,node_vec);
            let (gt,gd) = KDTree::build_helper(triangles,ga,depth+1,node_vec);
            let node: &mut KDNode = node_vec.index_mut(idx);
            match *node {
                KDNode::Branch(ref mut branch) => {
                    branch.child_lt = lt;
                    branch.child_gt = gt;
                },
                _ => panic!("Expected a Branch node!")
            }
            (idx,max(ld,gd))
        } else {
            let idx = node_vec.len();
            //println!("leaf at {}",idx);
            node_vec.push(KDNode::Leaf(KDLeaf {
                tri_inds: indices,
            }));
            (idx,depth)
        }
    }

    pub fn build(triangles: &Vec<Triangle>) -> KDTree {
        let mut mins = Vec3::new(f64::INFINITY);
        let mut maxs = Vec3::new(f64::NEG_INFINITY);
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
        /*let (la,ga) = aabb.split(biggest_axis,sval);
        KDTree {
            root: KDNode::Branch(KDBranch{
                aabb: aabb,
                split_axis: biggest_axis,
                split_val: sval,
            child_lt: KDTree::build_helper(triangles,la,0),
            child_gt: KDTree::build_helper(triangles,ga,0),
            }),
        }*/
        let mut vec = Vec::new();
        let (_,max_depth) = KDTree::build_helper(triangles,aabb,0, &mut vec);
        /*let flat_tree = Vec::with_capacity(count);
        let mut node = tree;
        let mut index = 0;
        loop {
            match *node {
                KDNode::Branch(branch) => {
                    flat_tree.append(KDFlatBranch{
                        aabb: branch.aabb,
                        split_axis: branch.split_axis,
                        split_val: branch.split_val,
                        ch

                    });
                }
            }
            
        }*/
        KDTree {
            root: vec,
            intersect_stack_size: max_depth
        }
    }

    /*fn flatten_helper(node: Box<KDNode>, index: usize, vec: &mut Vec<KDFlatNode>){
        match *node {
            KDNode::Branch(branch) => {
                let mut flat = KDFlatBranch{
                        aabb: branch.aabb,
                        split_axis: branch.split_axis,
                        split_val: branch.split_val,
                        child_lt: 0,
                        child_gt: 0
                    };
                    vec.append(KDNodeflat);

                }
    }*/

    pub fn intersect<'a>(&self, triangles: &'a Vec<Triangle>, ray: Ray, scratch_space: &mut Vec<usize>) -> (Option<&'a Triangle>, f64){
        let tree = &self.root;
        let mut stack = scratch_space;
        stack.push(0);
        loop {
            if let Some(index) = stack.pop() {
                match tree[index] {
                    KDNode::Branch(ref branch) => {
                        if branch.aabb.contains_ray(ray) {
                            stack.push(branch.child_lt);
                            stack.push(branch.child_gt);
                        }
                    },
                    KDNode::Leaf(ref leaf) => {
                        if let (Some(x),y) = intersect_selected_triangle(triangles,&leaf.tri_inds,ray) {
                            return (Some(x),y)
                        }
                    },
                }
            } else {
                break;
            }
        }
        (None,std::f64::INFINITY)
    }
}

pub fn load_mesh(path: &str) -> (Vec<Triangle>, KDTree) {
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
    println!("Loaded {}, {} triangles, kdtree has {} nodes",path,obj_tris.len(),kdtree.root.len());
    (obj_tris, kdtree)
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn aabb_split(){
        let base = AABB{
            x: (0.0, 1.0),
            y: (0.0, 1.0),
            z: (0.0, 1.0)
        };
        {
            let (lt,gt) = base.split(Axis::X,0.5);
            assert_eq!(lt.x, (0.0,0.5));
            assert_eq!(gt.x, (0.5,1.0));
        }
        {
            let (lt,gt) = base.split(Axis::Y,0.5);
            assert_eq!(lt.y, (0.0,0.5));
            assert_eq!(gt.y, (0.5,1.0));
        }
        {
            let (lt,gt) = base.split(Axis::Z,0.5);
            assert_eq!(lt.z, (0.0,0.5));
            assert_eq!(gt.z, (0.5,1.0));
        }
    }

    #[test]
    fn aabb_split_biggest(){
        let base = AABB{
            x: (0.0, 2.0),
            y: (0.0, 1.0),
            z: (0.0, 1.0)
        };
        let avg = Vec3::set(1.0,0.5,0.5);
        assert_eq!(base.biggest_dim(avg).0,Axis::X);
        let (lt,gt) = base.split_biggest(avg);
        assert_eq!(lt.x, (0.0,1.0));
        assert_eq!(gt.x, (1.0,2.0));
    }
    #[test]
    fn kdtree_build_single(){
        let single = vec![Triangle { points:[ 
            Vec3::set(0.0, 0.0, 0.0),  
            Vec3::set( 1.0,  1.0, 1.0),  
            Vec3::set(  0.5,  0.5, 0.5),
        ]}];
        let single_tree = KDTree::build(&single);
        match *single_tree.root {
            KDNode::Leaf(leaf) => {
                assert_eq!(leaf.tri_inds.len(),1);
                assert_eq!(leaf.tri_inds[0],0);
            },
            KDNode::Branch(branch) => {
                println!("{:?}", branch);
                panic!("Expected a Leaf KDNode!")
            }
        }
    }
}
