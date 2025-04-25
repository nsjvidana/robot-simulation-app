use std::ops::{Deref, DerefMut};
use crate::error::Error;

pub type Result<T> = core::result::Result<T, Error>;

pub type Real = f32;

pub struct W<T>(pub T);

impl<T> Deref for W<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for W<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}