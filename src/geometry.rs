use nalgebra::{Vector2, Vector3};
use serde::Deserialize;

/// A line in 2D space. Used to represent the gaze vector on the image plane.
/// The line is represented by a point on the line and a direction vector.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Line2D {
    pub origin: Vector2<f64>,
    pub direction: Vector2<f64>,
}

impl Line2D {
    pub fn new(origin: Vector2<f64>, direction: Vector2<f64>) -> Self {
        Line2D { origin, direction: direction.normalize() }
    }
}

/// An ellipse in 2D space. Used to represent the pupil on the image plane.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Ellipse2D {
    pub center: Vector2<f64>,
    pub major_radius: f64,
    pub minor_radius: f64,
    pub angle: f64,
}

/// A circle in 2D space. Used to represent the eye on the image plane.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Circle2D {
    pub center: Vector2<f64>,
    pub radius: f64,
}

/// A line in 3D space. Used to represent the gaze vector in 3D space.
/// The line is represented by a point on the line and a direction vector.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Line3D {
    pub origin: Vector3<f64>,
    pub direction: Vector3<f64>,
}

/// A circle in 3D space. Used to represent the pupil in 3D space.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Circle3D {
    pub center: Vector3<f64>,
    pub normal: Vector3<f64>,
    pub radius: f64,
}

/// A sphere in 3D space. Used to represent the eye in 3D space.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Sphere3D {
    pub center: Vector3<f64>,
    pub radius: f64,
}
