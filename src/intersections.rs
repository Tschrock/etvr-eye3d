//! This module contains functions for finding intersections between geometric objects.

use nalgebra::Vector3;

/// Finds the intersections of a line and a sphere.
///
/// See: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
///
/// # Arguments
/// * `line_origin` - The origin of the line.
/// * `line_direction` - The direction of the line.
/// * `sphere_center` - The center of the sphere.
/// * `sphere_radius` - The radius of the sphere.
///
/// # Returns
/// * `None` if there are no intersections.
/// * `Some((d1, d1))` if there is one intersection, where `d1` is the distance from the line origin.
/// * `Some((d1, d2))` if there are two intersections, where `d1` and `d2` are the distances from the
///  line origin.
///
pub fn intersect_line_with_sphere(
    line_origin: &Vector3<f64>,
    line_direction: &Vector3<f64>,
    sphere_center: &Vector3<f64>,
    sphere_radius: f64,
) -> Option<(f64, f64)> {
    let c = sphere_center;
    let r = sphere_radius;
    let o = line_origin;
    let u = line_direction;

    let oc = o - c;
    let dot_product = u.dot(&oc);
    let delta = dot_product * dot_product - oc.norm_squared() + r * r;
    if delta < 0.0 {
        return None;
    } else {
        let sqrt = delta.sqrt();
        let d1 = -dot_product + sqrt;
        let d2 = -dot_product - sqrt;
        Some((d1, d2))
    }
}

/// Finds the nearest point on a sphere to a line.
///
/// # Arguments
/// * `sphere_center` - The center of the sphere.
/// * `sphere_radius` - The radius of the sphere.
/// * `line_origin` - The origin of the line.
/// * `line_direction` - The direction of the line.
///
/// # Returns
/// The nearest point on the sphere to the line.
pub fn nearest_point_on_sphere_to_line(
    sphere_center: &Vector3<f64>,
    sphere_radius: f64,
    line_origin: &Vector3<f64>,
    line_direction: &Vector3<f64>,
) -> Vector3<f64> {
    // first check if the line intersects the sphere
    let intersection =
        intersect_line_with_sphere(line_origin, line_direction, sphere_center, sphere_radius);
    if let Some((d1, d2)) = intersection {
        // d is the distance from the origin of the line
        // we want the smallest d
        let d = d1.min(d2);
        // solve for the intersection point
        return line_origin + line_direction.scale(d);
    } else {
        // if not, the nearest point will be on the line that is perpendicular
        // to the original line and passes through the center of the sphere.
        // See https://www.geogebra.org/3d/udwvetej
        let distance = line_direction.dot(&(sphere_center - line_origin));
        let origin_prime = line_origin + line_direction.scale(distance);
        let direction_prime = (sphere_center - origin_prime).normalize();
        // This new line should intersect the sphere
        let intersection = intersect_line_with_sphere(
            &origin_prime,
            &direction_prime,
            sphere_center,
            sphere_radius,
        );
        if let Some((d1, d2)) = intersection {
            let d = d1.min(d2);
            return origin_prime + direction_prime.scale(d);
        } else {
            // Can this happen?
            Vector3::<f64>::zeros()
        }
    }
}
