use std::sync::Arc;
use std::cell::UnsafeCell;
use std::ops::{Deref, DerefMut};

pub struct UnorderedBufferWriter<T:Send + Sync> {
    buffer: Arc<UnsafeCell<Box<[T]>>>,
}

#[derive(Clone)] //Clone intentionally only implemented for the reader
pub struct UnorderedBufferReader<T:Send + Sync> {
    buffer: Arc<UnsafeCell<Box<[T]>>>,
}

pub fn new<T: Send + Sync + Clone>(size: usize, default_value: T) -> (UnorderedBufferWriter<T>,UnorderedBufferReader<T>) {
    let vec_buff = vec![default_value; size];
    let boxed = vec_buff.into_boxed_slice();
    let writer = UnorderedBufferWriter {
        buffer: Arc::new(UnsafeCell::new(boxed)),
    };
    let reader = UnorderedBufferReader {
        buffer: writer.buffer.clone(),
    };
    (writer, reader)
}

pub fn from_boxed_slice<T: Send + Sync>(boxed: Box<[T]>) -> (UnorderedBufferWriter<T>,UnorderedBufferReader<T>) {
    let writer = UnorderedBufferWriter {
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