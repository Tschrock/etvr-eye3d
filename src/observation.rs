use nalgebra::{Matrix2, Matrix2x3, Matrix3, Matrix3x4};

use crate::geometry::{Circle3D, Ellipse2D, Line2D};
use crate::projections::{project_line_into_image_plane, unproject_ellipse};
use crate::{TrackingError, EYE_RADIUS_DEFAULT};

/// A structure that holds an observation of a pupil alongside precomputed values for gaze estimation.
#[derive(Debug)]
pub struct Observation {
    /// The timestamp of the observation.
    pub timestamp: f64,
    /// The ellipse representing the pupil on the image plane.
    pub ellipse: Ellipse2D,
    /// The two possible unprojections of the pupil ellipse.
    pub unprojected_circles: [Circle3D; 2],
    /// The gaze line in 2D space.
    pub gaze_2d: Line2D,
    /// The auxiliary matrix used to compute the projected eye center.
    pub aux_2d: Matrix2x3<f64>,
    /// The auxiliary matrix used to compute the unprojected eye center.
    pub aux_3d: [Matrix3x4<f64>; 2],
}

impl Observation {
    pub fn new(
        timestamp: f64,
        ellipse: Ellipse2D,
        focal_length: f64,
    ) -> Result<Self, TrackingError> {
        // Unproject the pupil ellipse.
        let unprojected_circles = unproject_ellipse(&ellipse, focal_length, 1.0)?;

        // Reproject the gaze line.
        let gaze_2d = project_line_into_image_plane(
            unprojected_circles[0].center,
            unprojected_circles[0].normal,
            focal_length,
        );

        // Compute auxiliary matrices.
        let aux_2d = calc_aux_2d(&gaze_2d);
        let aux_3d = calc_aux_3d(&unprojected_circles);

        Ok(Observation {
            timestamp,
            ellipse,
            unprojected_circles,
            gaze_2d,
            aux_2d,
            aux_3d,
        })
    }
}

/// Computes the auxiliary matrix used to compute the projected eye center.
/// This corresponds to the two inner parts of equation (6) in [Swirski and Dodgson 2013].
/// 
/// ```math
/// c = (sumᵢ(I - nᵢnᵢᵀ))⁻¹(sumᵢ(I - nᵢnᵢᵀ)pᵢ)
/// ```
/// 
/// Specifically,
/// ```math
/// aux_2d[0..2, 0..2] = I - nᵢnᵢᵀ
/// ```
/// and
/// ```math
/// aux_2d[0..2, 2] = (I - nᵢnᵢᵀ)pᵢ
/// ```
fn calc_aux_2d(gaze_2d: &Line2D) -> Matrix2x3<f64> {
    let mut aux_2d = Matrix2x3::default();
    // I - nᵢnᵢᵀ
    let tmp = Matrix2::identity() - gaze_2d.direction * gaze_2d.direction.transpose();
    aux_2d.fixed_view_mut::<2, 2>(0, 0).copy_from(&tmp);
    // (I - nᵢnᵢᵀ)pᵢ
    aux_2d
        .fixed_view_mut::<2, 1>(0, 2)
        .copy_from(&(tmp * gaze_2d.origin));
    aux_2d
}

/// Computes the auxiliary matrix used to compute the unprojected eye center.
/// This corresponds to the two inner parts of equation (3) in [Dierkes et al. 2019]
/// for each of the two possible unprojections [i].
///
/// ```math
/// E = (sumᵢ(I - dᵢdᵢᵀ))⁻¹(sumᵢ(I - dᵢdᵢᵀ)(-Rnᵢ))
/// ```
///
/// Specifically,
/// ```math
/// aux_3d[i, 0..3, 0..3] = I - dᵢdᵢᵀ
/// ```
/// and
/// ```math
/// aux_3d[i, 0..3, 3] = (I - dᵢdᵢᵀ)(-Rnᵢ)
/// ```
fn calc_aux_3d(circles: &[Circle3D; 2]) -> [Matrix3x4<f64>; 2] {
    let mut aux_3d = [Matrix3x4::default(), Matrix3x4::default()];
    for i in 0..2 {
        let circle = &circles[i];
        // -Rnᵢ
        let origin = circle.center - EYE_RADIUS_DEFAULT * circle.normal;
        let direction = circle.center.normalize();
        // I - dᵢdᵢᵀ
        let tmp = Matrix3::identity() - direction * direction.transpose();
        aux_3d[i].fixed_view_mut::<3, 3>(0, 0).copy_from(&tmp);
        // (I - dᵢdᵢᵀ)(-Rnᵢ)
        aux_3d[i]
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(tmp * origin));
    }
    aux_3d
}
