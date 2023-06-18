use std::rc::Rc;

use nalgebra::{Matrix2x3, Matrix3, Matrix3x4, Vector2, Vector3};

use crate::cache::ObservationCache;
use crate::geometry::Circle3D;
use crate::intersections::nearest_point_on_sphere_to_line;
use crate::observation::Observation;
use crate::{TrackingError, EYE_RADIUS_DEFAULT};

pub struct EyeModel<T: ObservationCache> {
    /// A cache of recent observations.
    observations: T,
    /// The center of the eye on the image plane.
    pub projected_eye_center: Vector2<f64>,
    /// The center of the eye in 3D space.
    pub unprojected_eye_center: Vector3<f64>,
}

impl<T: ObservationCache> EyeModel<T> {
    pub fn new(cache: T) -> Self {
        EyeModel {
            observations: cache,
            projected_eye_center: Vector2::new(0.0, 0.0),
            unprojected_eye_center: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    /// Adds a new observation to the model.
    pub fn add(&mut self, observation: Rc<Observation>) {
        self.observations.push(observation);
    }

    /// Updates the estimated eye center to account for new observations.
    ///
    /// # Arguments
    /// 
    /// * `projected` - The projected (2d) eye center.
    /// * `unprojected` - The unprojected (3d) eye center.
    /// * `unprojected_certainty` - The certainty of the unprojected eye center.
    pub fn update_eye_center(
        &mut self,
        projected: Option<Vector2<f64>>,
        unprojected: Option<Vector3<f64>>,
        unprojected_certainty: Option<f64>,
    ) -> Result<(), TrackingError> {
        // Calculate the projected (2D) eye center.
        self.projected_eye_center = match projected {
            Some(center) => center,
            None => self.calc_projected_eye_center()?,
        };

        // Calculate the unprojected (3D) eye center.
        self.unprojected_eye_center = self.calc_unprojected_eye_center(
            self.projected_eye_center,
            unprojected,
            unprojected_certainty,
        )?;

        Ok(())
    }

    /// Calculates the projected eye center.
    fn calc_projected_eye_center(&self) -> Result<Vector2<f64>, TrackingError> {
        // aux_2d is a 2x3 matrix that represents the inner two parts of
        // [Swirski and Dodgson 2013, eq. 6]

        // The first half, aux_2d[0..2, 0..2], is
        // I - nᵢnᵢᵀ

        // The second half, aux_2d[0..2, 2], is
        // (I - nᵢnᵢᵀ)pᵢ

        // Since this is the projected eye center, we don't need to
        // disambiguate between the two possible pupil circles, and
        // can just sum the aux_2d matrices for each observation.
        let sum_aux_2d: Matrix2x3<f64> = self.observations.iter().map(|o| o.aux_2d).sum();

        // Split the parts of the matrix into their own variables.
        let matrix = sum_aux_2d.fixed_view::<2, 2>(0, 0);
        let vector = sum_aux_2d.fixed_view::<2, 1>(0, 2);

        // Then we can finish the calculation of the unprojected eye center.
        // c = (sumᵢ(I - nᵢnᵢᵀ))⁻¹(sumᵢ(I - nᵢnᵢᵀ)pᵢ)
        Ok(matrix
            .pseudo_inverse(1e-15)
            .map_err(|_| TrackingError::MatrixInversionFailed)?
            * vector)
    }

    /// Calculates the unprojected eye center.
    /// If the caller provides a 3D center, it will be mixed into the calculation.
    /// 
    /// # Arguments
    /// 
    /// * `center_2d` - The projected eye center.
    /// * `center_3d` - The unprojected eye center, if known.
    /// * `center_3d_certainty` - The certainty of the unprojected eye center.
    ///   Both must be provided or neither will be used.
    /// 
    /// # Returns
    /// 
    /// The unprojected eye center.
    fn calc_unprojected_eye_center(
        &self,
        center_2d: Vector2<f64>,
        center_3d: Option<Vector3<f64>>,
        center_3d_certainty: Option<f64>,
    ) -> Result<Vector3<f64>, TrackingError> {
        // aux_3d is a 2x3x4 matrix that represents the inner two parts of
        // [Dierkes et al. 2019, eq. 3] for the two possible pupil circles [i].

        // The first half, aux_3d[i, 0..3, 0..3], is
        // I - dᵢdᵢᵀ

        // The second half, aux_3d[i, 0..3, 3], is
        // (I - dᵢdᵢᵀ)(-Rnᵢ)

        // We need to disambiguate between the two possible pupil circles, and
        // then sum the aux_3d matrices for each observation.
        let sum_aux_3d: Matrix3x4<f64> = self
            .observations
            .iter()
            .map(|observation| {
                // get the normal that is facing the center of the eye
                let towards_center = Vector2::new(
                    center_2d[0] - observation.gaze_2d.origin[0],
                    center_2d[1] - observation.gaze_2d.origin[1],
                );
                // The dot product will tell us if gaze_2d is pointing towards
                // the eye center, which tells us which aux_3d to use.
                let dot_product = towards_center.dot(&observation.gaze_2d.direction);
                if dot_product < 0.0 {
                    observation.aux_3d[0]
                } else {
                    observation.aux_3d[1]
                }
            })
            .sum();

        // Split the parts of the matrix into their own variables.
        let matrix = sum_aux_3d.fixed_view::<3, 3>(0, 0);
        let vector = sum_aux_3d.fixed_view::<3, 1>(0, 3);

        // Then we can finish the calculation of the unprojected eye center.
        // E = (sumᵢ(I - dᵢdᵢᵀ))⁻¹(sumᵢ(I - dᵢdᵢᵀ)(-Rnᵢ))

        // Also, if the caller provided a 3D center, mix it into the calculation.
        // This feels verbose, but idk a more concise way to do it in rust.
        let center = if let (Some(center_3d), Some(center_3d_certainty)) =
            (center_3d, center_3d_certainty)
        {
            let matrix = matrix + center_3d_certainty * Matrix3::identity();
            let vector = vector + center_3d_certainty * center_3d;
            matrix
                .try_inverse()
                .ok_or(TrackingError::MatrixInversionFailed)?
                * vector
        } else {
            matrix
                .try_inverse()
                .ok_or(TrackingError::MatrixInversionFailed)?
                * vector
        };

        Ok(center)
    }
    /// Calculates the mean observation circularity.
    pub fn mean_observation_circularity(&self) -> f64 {
        self.observations
            .iter()
            .map(|o| o.ellipse.minor_radius / o.ellipse.major_radius)
            .sum::<f64>()
            / self.observations.len() as f64
    }
    /// Calculates the pupil circle for the given observation.
    pub fn get_pupil_circle(&self, observation: &Observation) -> Circle3D {
        let circle_3d = self.disambiguate_circle_3d_pair(observation);
        let unprojection_depth = circle_3d.center.norm();
        let direction = circle_3d.center / unprojection_depth;

        let nearest_point_on_sphere = nearest_point_on_sphere_to_line(
            &self.unprojected_eye_center,
            EYE_RADIUS_DEFAULT,
            &Vector3::new(0.0, 0.0, 0.0),
            &direction,
        );

        let gaze_vector = (nearest_point_on_sphere - self.unprojected_eye_center).normalize();

        let radius = nearest_point_on_sphere.norm() / unprojection_depth;
        Circle3D {
            center: nearest_point_on_sphere,
            normal: gaze_vector,
            radius,
        }
    }
    fn disambiguate_circle_3d_pair<'a>(&self, observation: &'a Observation) -> &'a Circle3D {
        let circle_center_2d = observation.ellipse.center;
        let circle_normal_2d = observation.gaze_2d.direction.normalize();
        let sphere_center_2d = self.projected_eye_center;
        if (circle_center_2d - sphere_center_2d).dot(&circle_normal_2d) >= 0.0 {
            &observation.unprojected_circles[0]
        } else {
            &observation.unprojected_circles[1]
        }
    }
    /// Applies refraction correction to the given pupil circle.
    pub fn apply_refraction_correction(
        &mut self,
        pupil_circle: &Circle3D,
    ) -> (Vector3<f64>, Circle3D) {
        // TODO
        (
            self.unprojected_eye_center.clone_owned(),
            pupil_circle.clone(),
        )
    }
}
