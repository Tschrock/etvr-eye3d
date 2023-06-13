use nalgebra::{Vector2, Vector3};
use serde::Deserialize;

#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Circle3D {
    pub center: Vector3<f64>,
    pub normal: Vector3<f64>,
    pub radius: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
pub struct Ellipse2D {
    pub center: Vector2<f64>,
    pub major_radius: f64,
    pub minor_radius: f64,
    pub angle: f64,
}
