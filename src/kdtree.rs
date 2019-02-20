use std;
use std::f64;
use std::path::Path;
use std::ops::IndexMut;
use std::cmp::Ordering;

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
        
        /*if self.x.0 > self.x.1 || self.y.0 > self.y.1 || self.z.0 > self.z.1 {
            panic!("AABB invalid!");
        }*/
        /*let mut tmin = -std::f64::INFINITY;
        let mut tmax = std::f64::INFINITY;
        if ray.direction.x != 0.0 {
            let tx1 = (self.x.0 - ray.origin.x)/ray.direction.x;
            let tx2 = (self.x.1 - ray.origin.x)/ray.direction.x;
            tmin = f64::max(tmin, f64::min(tx1,tx2));
            tmax = f64::min(tmax, f64::max(tx1,tx2));
        }
        if ray.direction.y != 0.0 {
            let ty1 = (self.y.0 - ray.origin.y)/ray.direction.y;
            let ty2 = (self.y.1 - ray.origin.y)/ray.direction.y;
            tmin = f64::max(tmin, f64::min(ty1,ty2));
            tmax = f64::min(tmax, f64::max(ty1,ty2));
        }
        if ray.direction.z != 0.0 {
            let tz1 = (self.z.0 - ray.origin.z)/ray.direction.z;
            let tz2 = (self.z.1 - ray.origin.z)/ray.direction.z;
            tmin = f64::max(tmin, f64::min(tz1,tz2));
            tmax = f64::min(tmax, f64::max(tz1,tz2));
        }
        tmax >= tmin*/

        // r.dir is unit direction vector of ray
        let dirfrac = Vec3::set(1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z);
        // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
        // r.org is origin of ray
        let t1 = (self.x.0 - ray.origin.x)*dirfrac.x;
        let t2 = (self.x.1 - ray.origin.x)*dirfrac.x;
        let t3 = (self.y.0 - ray.origin.y)*dirfrac.y;
        let t4 = (self.y.1 - ray.origin.y)*dirfrac.y;
        let t5 = (self.z.0 - ray.origin.z)*dirfrac.z;
        let t6 = (self.z.1 - ray.origin.z)*dirfrac.z;

        let tmin = f64::max(f64::max(f64::min(t1, t2), f64::min(t3, t4)), f64::min(t5, t6));
        let tmax = f64::min(f64::min(f64::max(t1, t2), f64::max(t3, t4)), f64::max(t5, t6));

        // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
        if tmax < 0.0 {
            return false;
        }

        // if tmin > tmax, ray doesn't intersect AABB
        if tmin > tmax {
            return false;
        }

        return true;
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
    fn area(&self) -> f64 {
        (self.x.1 - self.x.0) * (self.y.1 - self.y.0) * (self.z.1 - self.z.0)
    }
    fn contains_tri(&self, tri: &Triangle) -> bool {
        //The naive check of the corners is _not_ correct
        if self.contains_point(&tri.points[0]) || self.contains_point(&tri.points[1]) || self.contains_point(&tri.points[2]) {
            return true;
        }
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
                    idx += 1;
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
            mins.z = f64::min(mins.z, pt.z);
            maxs.x = f64::max(maxs.x, pt.x);
            maxs.y = f64::max(maxs.y, pt.y);
            maxs.z = f64::max(maxs.z, pt.z);
        }
        AABB {
            x: (mins.x,maxs.x),
            y: (mins.y,maxs.y),
            z: (mins.z,maxs.z)
        }
    }
    pub fn union(&self, other: &AABB) -> AABB {
        let mut mins = Vec3::new(f64::INFINITY);
        let mut maxs = Vec3::new(f64::NEG_INFINITY);
        mins.x = f64::min(self.x.0, other.x.0);
        mins.y = f64::min(self.y.0, other.y.0);
        mins.z = f64::min(self.z.0, other.z.0);
        maxs.x = f64::max(self.x.1, other.x.1);
        maxs.y = f64::max(self.y.1, other.y.1);
        maxs.z = f64::max(self.z.1, other.z.1);
        AABB {
            x: (mins.x,maxs.x),
            y: (mins.y,maxs.y),
            z: (mins.z,maxs.z)
        }
    }

    pub fn infinite() -> AABB {
        let mut mins = Vec3::new(f64::NEG_INFINITY);
        let mut maxs = Vec3::new(f64::INFINITY);
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

impl Axis {
    pub fn from_idx(idx: usize) -> Axis {
        match idx {
            0 => Axis::X,
            1 => Axis::Y,
            2 => Axis::Z,
            _ => panic!("Axis index {} out of bounds",idx)
        }
    }
}

#[derive(Debug)]
struct KDBranch {
    aabb: AABB,
    split_axis: Axis,
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
    fn build_helper2_delegate<'a, 'b>(axis: usize, triangles: &'b [(Triangle,usize)], sorted: &'a mut Vec<(Triangle,usize)>) -> ((&'a [(Triangle,usize)],&'a [(Triangle,usize)]),(AABB,AABB)) {
        //let mut sorted: Vec<(Triangle,usize)> = Vec::new();
        sorted.extend(triangles);
        sorted.sort_unstable_by(|left,right|{
            let lbb = AABB::from_tri(&left.0);
            let rbb = AABB::from_tri(&right.0);
            let (lmin,lmax) = lbb.corners();
            let (rmin,rmax) = rbb.corners();
            if lmin[axis] > rmax[axis] || lmax[axis] > rmax[axis] && lmin[axis] > rmin[axis] {
                Ordering::Greater
            } else if lmax[axis] < rmin[axis] || lmax[axis] < rmax[axis] && lmin[axis] < rmin[axis] {
                Ordering::Less
            } else {
                Ordering::Equal
            }
        });
        let (left,right) = sorted.split_at(sorted.len()/2);
        let lbb = left.iter().map(|tri|AABB::from_tri(&tri.0)).fold(None,|macc: Option<AABB>, bb|{
            if let Some(acc) = macc {
                Some(acc.union(&bb))
            } else {
                Some(bb)
            }
        }).unwrap();
        let rbb = right.iter().map(|tri|AABB::from_tri(&tri.0)).fold(None,|macc: Option<AABB>, bb|{
            if let Some(acc) = macc {
                Some(acc.union(&bb))
            } else {
                Some(bb)
            }
        }).unwrap();
        ((left,right),(lbb,rbb))
    }
    fn build_helper2(triangles: &[(Triangle,usize)], aabb:AABB, depth: usize, node_vec: &mut Vec<KDNode>) -> (usize,usize) {
        if triangles.len() < 2  {
            let idx = node_vec.len();
            //println!("leaf with {:?}",triangles);
            node_vec.push(KDNode::Leaf(KDLeaf {
                tri_inds: triangles.iter().map(|(t,u)|*u).collect(),
            }));
            (idx,depth)
        } else {
            /*let axis = depth % 3;
            let mut sorted: Vec<(Triangle,usize)> = Vec::new();
            sorted.extend(triangles);
            sorted.sort_unstable_by(|left,right|{
                let lbb = AABB::from_tri(&left.0);
                let rbb = AABB::from_tri(&right.0);
                let (lmin,lmax) = lbb.corners();
                let (rmin,rmax) = rbb.corners();
                if lmin[axis] > rmax[axis] || lmax[axis] > rmax[axis] && lmin[axis] > rmin[axis] {
                    Ordering::Greater
                } else if lmax[axis] < rmin[axis] || lmax[axis] < rmax[axis] && lmin[axis] < rmin[axis] {
                    Ordering::Less
                } else {
                    Ordering::Equal
                }
            });
            let (left,right) = sorted.split_at(sorted.len()/2);
            let lbb = left.iter().map(|tri|AABB::from_tri(&tri.0)).fold(None,|macc: Option<AABB>, bb|{
                if let Some(acc) = macc {
                    Some(acc.union(&bb))
                } else {
                    Some(bb)
                }
            }).unwrap();
            let rbb = right.iter().map(|tri|AABB::from_tri(&tri.0)).fold(None,|macc: Option<AABB>, bb|{
                if let Some(acc) = macc {
                    Some(acc.union(&bb))
                } else {
                    Some(bb)
                }
            }).unwrap();
            let chosen_axis = axis;*/
            let mut xvec = Vec::new();
            let mut yvec = Vec::new();
            let mut zvec = Vec::new();
            let ((mut left,mut right),(mut lbb,mut rbb)) = Self::build_helper2_delegate(0,triangles,&mut xvec);
            let mut area = lbb.area() + rbb.area();
            let mut chosen_axis = 0;

            let ((l,r),(lb,rb)) = Self::build_helper2_delegate(1,triangles,&mut yvec);
            let new_area = lb.area() + rb.area();
            if new_area < area {
                chosen_axis = 1;
                area = new_area;
                left = l;
                right = r;
                lbb = lb;
                rbb = rb;
            }

            let ((l,r),(lb,rb)) = Self::build_helper2_delegate(2,triangles,&mut zvec);
            let new_area = lb.area() + rb.area();
            if new_area < area {
                chosen_axis = 2;
                area = new_area;
                left = l;
                right = r;
                lbb = lb;
                rbb = rb;
            }
            
            let idx = node_vec.len();
            node_vec.push(KDNode::Branch(KDBranch{
                aabb: aabb,
                split_axis: Axis::from_idx(chosen_axis),
                child_lt: 0,
                child_gt: 0,
                }));
            let (lt,ld) = KDTree::build_helper2(left,lbb,depth+1,node_vec);
            let (gt,gd) = KDTree::build_helper2(right,rbb,depth+1,node_vec);
            let node: &mut KDNode = node_vec.index_mut(idx);
            match *node {
                KDNode::Branch(ref mut branch) => {
                    branch.child_lt = lt;
                    branch.child_gt = gt;
                },
                _ => panic!("Expected a Branch node!")
            }
            (idx,std::cmp::max(ld,gd))
        }
    }
    /**
     * returns a tuple where the first element is the index of the root of this subtree and the second is the maximum depth encountered
     */
    fn build_helper(triangles: &Vec<MeshTriangle>, aabb:AABB, depth: usize, node_vec: &mut Vec<KDNode>) -> (usize,usize) {
        let mut sum = Vec3::zero();
        let mut count:u32 = 0;
        let mut indices = Vec::new();
        for i in 0..triangles.len() {
            let ref tri = triangles[i];
            if aabb.contains_tri(&tri.pos) /* aabb.intersects(&AABB::from_tri(&tri)) */ {
                indices.push(i);
                for pt in tri.pos.iter() {
                    sum = sum + *pt;
                    count += 1;
                }
            }
        }
        let (biggest_axis, sval) = aabb.biggest_dim(sum * (1.0/(count as f64)));
        let (la, ga) = aabb.split_biggest(sum * (1.0/(count as f64)));
        //println!("count:{} sval:{} axis:{:?} la:{:?} ga:{:?}",count,sval,biggest_axis,la,ga);
        if depth < 1000 && count > 2 && la.valid() && ga.valid() { //arbitrarily picked 20 triangles as leaf size
            let idx = node_vec.len();
            //println!("branch at {}",idx);
            node_vec.push(KDNode::Branch(KDBranch{
                aabb: aabb,
                split_axis: biggest_axis,
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
            (idx,std::cmp::max(ld,gd))
        } else {
            let idx = node_vec.len();
            //println!("leaf at {}",idx);
            node_vec.push(KDNode::Leaf(KDLeaf {
                tri_inds: indices,
            }));
            (idx,depth)
        }
    }

    pub fn build(triangles: &Vec<MeshTriangle>) -> KDTree {
        let mut mins = Vec3::new(f64::INFINITY);
        let mut maxs = Vec3::new(f64::NEG_INFINITY);
        let mut sum = Vec3::zero();
        for tri in triangles.iter() {
            for pt in tri.pos.iter() {
                mins.x = f64::min(mins.x,pt.x);
                mins.y = f64::min(mins.y,pt.y);
                mins.z = f64::min(mins.z,pt.z);
                maxs.x = f64::max(maxs.x,pt.x);
                maxs.y = f64::max(maxs.y,pt.y);
                maxs.z = f64::max(maxs.z,pt.z);
                sum = sum + *pt;
            }
        }
        let aabb = AABB {
            x:(mins.x,maxs.x),
            y:(mins.y,maxs.y),
            z:(mins.z,maxs.z),
        };
        let mut vec = Vec::new();
        //let (_,max_depth) = KDTree::build_helper(triangles,aabb,0, &mut vec);
        let mut tri_indexes = Vec::with_capacity(triangles.len());
        for i in 0..triangles.len() {
            tri_indexes.push((triangles[i].pos,i));
        }
        let (_,max_depth) = KDTree::build_helper2(&tri_indexes,aabb,0, &mut vec);

        KDTree {
            root: vec,
            intersect_stack_size: max_depth
        }
    }

    fn intersect2helper<'a>(tree: &Vec<KDNode>, node: usize, triangles: &'a [MeshTriangle], ray: Ray) -> (Option<&'a MeshTriangle>, f64, Vec3){
        match tree[node] {
            KDNode::Branch(ref branch) => {
                if branch.aabb.contains_ray(ray) {
                    let lt = Self::intersect2helper(tree,branch.child_lt, triangles, ray);
                    let gt = Self::intersect2helper(tree,branch.child_gt, triangles, ray);
                    if lt.1 < gt.1 {
                        lt
                    } else {
                        gt
                    }
                } else {
                    (None,std::f64::INFINITY, Vec3::zero())
                }
            },
            KDNode::Leaf(ref leaf) => {
                intersect_selected_triangle(triangles,&leaf.tri_inds,ray)
            },
        }
    }

    pub fn intersect2<'a>(&self, triangles: &'a [MeshTriangle], ray: Ray, _scratch_space: &mut Vec<usize>) -> (Option<&'a MeshTriangle>, f64, Vec3){
        if let KDNode::Branch(ref branch) = &self.root[0] {
            if !branch.aabb.contains_ray(ray) {
                return (None,std::f64::INFINITY, Vec3::zero());
            }
        }
        let tree = &self.root;
        Self::intersect2helper(tree,0,triangles,ray)
    }

    pub fn verify(&self, triangles: &[(Triangle,usize)]) {
        println!("KDTree has {} nodes", self.root.len());
        let mut found = vec![-1; triangles.len()];
        let mut i = 0;
        for node in &self.root {
            match node {
                KDNode::Branch(ref _branch) => (),
                KDNode::Leaf(ref leaf) => {
                    for tri in &leaf.tri_inds {
                        if found[*tri] != -1 {
                            //println!("triangle {} in leafs {} and {}", tri, found[*tri], i);
                        } else {
                            found[*tri] = i as isize;
                        }
                    }
                }
            }
            i += 1;
        }
        i = 0;
        for pos in found {
            if pos == -1 {
                panic!("triangle {} not in a leaf!", i);
            }
            i += 1;
        }
        println!("all {} triangles present in tree", triangles.len());
    }
}

pub fn load_mesh(path: &str) -> (Vec<MeshTriangle>,Vec<tobj::Material>) {
    let (obj, materials) = tobj::load_obj(&Path::new(path)).unwrap();
    (obj.into_iter().flat_map(|part| {
        let mut prims = Vec::new();
        
        for f in 0..part.mesh.indices.len() / 3 {
            let mut tri = Vec::new();
            let mut texcoords = [[0.0f32; 2]; 3];
            let mut i = 0;
            for ind in &part.mesh.indices[3*f..3*f+3] {
                let x = part.mesh.positions[(3* *ind) as usize]as f64;
                let y = part.mesh.positions[(3* *ind) as usize +1]as f64;
                let z = part.mesh.positions[(3* *ind) as usize +2]as f64;
                let v = Vec3::set(x,y,z);
                if part.mesh.texcoords.len() > 0 {
                    texcoords[i] = [part.mesh.texcoords[(2* *ind) as usize],part.mesh.texcoords[(2* *ind) as usize + 1]];
                }
                tri.push(v * 2.0);
                i += 1;
            }
            prims.push(MeshTriangle{ pos: Triangle::from_vec3_slice(&tri), texcoord: texcoords, material_index: part.mesh.material_id.unwrap_or(0)})
        }
        prims
    }).collect(), materials)
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
    /*#[test]
    fn kdtree_build_single(){
        let single = vec![(Triangle { points:[ 
            Vec3::set(0.0, 0.0, 0.0),  
            Vec3::set( 1.0,  1.0, 1.0),  
            Vec3::set(  0.5,  0.5, 0.5),
        ]},0)];
        let single_tree = KDTree::build(&single);
        match single_tree.root[0] {
            KDNode::Leaf(ref leaf) => {
                assert_eq!(leaf.tri_inds.len(),1);
                assert_eq!(leaf.tri_inds[0],0);
            },
            KDNode::Branch(ref branch) => {
                println!("{:?}", branch);
                panic!("Expected a Leaf KDNode!")
            }
        }
    }*/
}
