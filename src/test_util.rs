use crate::geometry::{Circle3D, Ellipse2D};
use nalgebra::{Dim, Matrix, Scalar, Storage};
use std::fmt::Display;

// Loads test data from a JSON file and stores it in a static variable.
macro_rules! test_data {
    ($path:literal as $($name:ident : $input:ty => $output:ty),+ $(,)?) => {
        #[derive(serde::Deserialize)]
        struct _TestData {
            $($name: Vec<($input, $output)>),+
        }
        lazy_static! {
            static ref TEST_DATA: _TestData = serde_json::from_str(include_str!($path)).unwrap();
        }
    };
}

// Can't use PartialEq because of floating point issues, can't use crates
// like float_eq because they don't do deep equals and we wouldn't be able
// to impl their traits on nalgebra types. Guess we'll just make our own.
// TODO: Add an option to specify the precision and comparison method.
pub enum FloatEqError {
    NotEqual,
    NotNear(Box<dyn Display>),
}

pub trait FloatEq<Rhs: ?Sized = Self>: PartialEq<Rhs> {
    fn float_eq(&self, other: &Rhs) -> Result<(), FloatEqError> {
        self.eq(other).then_some(()).ok_or(FloatEqError::NotEqual)
    }
}

impl FloatEq for f32 {
    fn float_eq(&self, other: &f32) -> Result<(), FloatEqError> {
        expect_float_relative_eq!(*self, *other, 0.00001)
            .map_err(|e| FloatEqError::NotNear(Box::new(e)))
    }
}

impl FloatEq for f64 {
    fn float_eq(&self, other: &f64) -> Result<(), FloatEqError> {
        expect_float_relative_eq!(*self, *other, 0.00001)
            .map_err(|e| FloatEqError::NotNear(Box::new(e)))
    }
}

// Default == impls
macro_rules! impl_default {
    ($($T:ident),+) => { $(impl FloatEq for $T {})+ };
}
impl_default!(bool, i32, i64, u32, u64, usize);

// Impl for structs based on their properties
macro_rules! impl_via_props {
    ($({$($t:ident),+},)? $T:ty: $($prop:tt),+) => {
        impl$(<$($t: FloatEq),*>)? FloatEq for $T {
            fn float_eq(&self, other: &$T) -> Result<(), FloatEqError> {
                $(self.$prop.float_eq(&other.$prop)?;)+
                Ok(())
            }
        }
    };
}
impl_via_props!(Circle3D: center, normal, radius);
impl_via_props!(Ellipse2D: center, major_radius, minor_radius, angle);
impl_via_props!({T1}, (T1,): 0);
impl_via_props!({T1, T2}, (T1, T2): 0, 1);
impl_via_props!({T1, T2, T3}, (T1, T2, T3): 0, 1, 2);
impl_via_props!({T1, T2, T3, T4}, (T1, T2, T3, T4): 0, 1, 2, 3);
impl_via_props!({T1, T2, T3, T4, T5}, (T1, T2, T3, T4, T5): 0, 1, 2, 3, 4);

// Impl for vecs
impl<T: FloatEq> FloatEq for Vec<T> {
    fn float_eq(&self, other: &Vec<T>) -> Result<(), FloatEqError> {
        self.len().float_eq(&other.len())?;
        self.iter()
            .zip(other.iter())
            .try_for_each(|(l, r)| l.float_eq(r))
    }
}

// Impl for arrays
impl<T: FloatEq, const S: usize> FloatEq for [T; S] {
    fn float_eq(&self, other: &[T; S]) -> Result<(), FloatEqError> {
        self.iter()
            .zip(other.iter())
            .try_for_each(|(l, r)| l.float_eq(r))
    }
}

// Impl for Matrices
impl<T: Scalar + FloatEq, R: Dim, C: Dim, S: Storage<T, R, C>> FloatEq for Matrix<T, R, C, S> {
    fn float_eq(&self, other: &Matrix<T, R, C, S>) -> Result<(), FloatEqError> {
        self.shape().float_eq(&other.shape())?;
        self.iter()
            .zip(other.iter())
            .try_for_each(|(l, r)| l.float_eq(r))
    }
}

// Impl for Option
impl<T: FloatEq> FloatEq for Option<T> {
    fn float_eq(&self, other: &Option<T>) -> Result<(), FloatEqError> {
        match (self, other) {
            (Some(l), Some(r)) => l.float_eq(r),
            (None, None) => Ok(()),
            _ => Err(FloatEqError::NotEqual),
        }
    }
}

// ^ Those could probablay all be handled by a iterator impl?

// The actual assert macro
macro_rules! assert_float_eq {
    ($left:expr, $right:expr $(,)?) => {
        let (left_val, right_val) = ($left, $right);
        match left_val.float_eq(&right_val) {
            Ok(_) => {}
            Err(FloatEqError::NotEqual) => {
                panic!(
                    "assertion failed: `(left == right)`\n     left: `{:?}`\n    right: `{:?}`",
                    left_val, right_val
                );
            }
            Err(FloatEqError::NotNear(e)) => {
                panic!(
                    "assertion failed: `(left == right)`\n     left: `{:?}`\n    right: `{:?}`\ncaused by:\n    {}",
                    left_val, right_val, e
                );
            }
        }
    };
}
