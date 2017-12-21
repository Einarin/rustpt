use std::sync::Arc;
use std::cell::UnsafeCell;
use std::ops::{Deref, DerefMut};
use vec3::*;

pub struct UnorderedBufferWriter<T:Send + Sync> {
    buffer: Arc<UnsafeCell<Box<[T]>>>,
}

pub struct UnorderedBufferReader<T:Send + Sync> {
    buffer: Arc<UnsafeCell<Box<[T]>>>,
}

pub fn new(size: usize) -> (UnorderedBufferWriter<Vec3>,UnorderedBufferReader<Vec3>) {
    let mut vec_buff = vec![ Vec3::zero(); size];
    let mut boxed = vec_buff.into_boxed_slice();
    let mut writer = UnorderedBufferWriter {
        buffer: Arc::new(UnsafeCell::new(boxed)),
    };
    let reader = UnorderedBufferReader {
        buffer: writer.buffer.clone(),
    };
    (writer, reader)
}

unsafe impl<T: Send + Sync> Send for UnorderedBufferWriter<T> { }
unsafe impl<T: Send + Sync> Send for UnorderedBufferReader<T> { }

impl<T: Send + Sync> Deref for UnorderedBufferReader<T> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        unsafe {
            &*self.buffer.get() as &[T]
        }
    }
}

impl<T: Send + Sync> Deref for UnorderedBufferWriter<T> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        unsafe {
            &*self.buffer.get() as &[T]
        }
    }
}

impl<T: Send + Sync> DerefMut for UnorderedBufferWriter<T> {

    fn deref_mut(&mut self) -> &mut [T] {
        unsafe {
            &mut *self.buffer.get() as &mut Box<[T]>
        }
    }
}