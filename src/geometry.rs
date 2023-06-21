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
    /// The center of the ellipse.
    /// The origin is the center of the image, with the x-axis pointing to the right, and the y-axis pointing down.
    pub center: Vector2<f64>,
    /// The semi-major axis of the ellipse.
    pub major_radius: f64,
    /// The semi-minor axis of the ellipse.
    pub minor_radius: f64,
    /// The angle of rotation of the ellipse, in radians.
    pub angle: f64,
}

impl Ellipse2D {
    /// Create a new ellipse.
    ///
    /// # Arguments
    ///
    /// * `x` - The x-coordinate of the center of the ellipse.
    /// * `y` - The y-coordinate of the center of the ellipse.
    /// * `major_radius` - The semi-major axis of the ellipse.
    /// * `minor_radius` - The semi-minor axis of the ellipse.
    /// * `angle` - The angle of rotation of the ellipse, in radians.
    ///
    pub fn new(x: f64, y: f64, major_radius: f64, minor_radius: f64, angle: f64) -> Self {
        if major_radius < minor_radius {
            Ellipse2D {
                center: Vector2::new(x, y),
                major_radius: minor_radius,
                minor_radius: major_radius,
                angle: angle + std::f64::consts::FRAC_PI_2,
            }
        } else {
            Ellipse2D {
                center: Vector2::new(x, y),
                major_radius,
                minor_radius,
                angle,
            }
        }
    }
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
