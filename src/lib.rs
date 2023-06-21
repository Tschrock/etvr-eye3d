#[cfg(test)]
#[macro_use]
extern crate assert_float_eq;

#[cfg(test)]
#[macro_use]
extern crate lazy_static;

#[cfg(test)]
#[macro_use]
mod test_util;

mod cache;
mod geometry;
mod intersections;
mod model;
mod observation;
mod projections;
mod timer;

use std::rc::Rc;

use cache::{FixedCache, SpacialCache};
use model::EyeModel;
use observation::Observation;
use thiserror::Error;

pub use geometry::{Circle2D, Circle3D, Ellipse2D, Sphere3D};
use projections::{
    project_circle_into_image_plane, project_sphere_into_image_plane, UnprojectionError,
};
use timer::{TimedUpdateController, UpdateController};

// This is basically a copy of Ellipse2D, but with clarification on the origin of the coordinate system.
// Should probably just use Ellipse2D instead.
/// The input to the eye tracker.
/// All units are in pixels.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PupilEllipse {
    /// The center of the ellipse, in the image's coordinate system.
    /// The origin is the top-left corner of the image, with the x-axis
    /// pointing to the right, and the y-axis pointing down.
    pub center: (f64, f64),
    /// The semi-major axis of the ellipse.
    pub major_radius: f64,
    /// The semi-minor axis of the ellipse.
    pub minor_radius: f64,
    /// The angle of rotation of the ellipse, in radians. The angle is measured
    /// from the positive horizontal axis to the ellipse's major axis.
    pub angle: f64,
}

impl PupilEllipse {
    /// Create a new pupil ellipse.
    /// The x and y coordinates are in the image's coordinate system, with the origin
    /// at the top-left corner of the image.
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
        PupilEllipse {
            center: (x, y),
            major_radius,
            minor_radius,
            angle,
        }
    }
}

/// An error that can occur during gaze estimation.
/// TODO: Should these be more context-specific?
#[derive(Error, Debug)]
pub enum TrackingError {
    // Matrix inversion failed.
    #[error("Matrix inversion failed.")]
    MatrixInversionFailed,
    // Pupil unprojection failed.
    #[error("Unprojection failed: {0}")]
    UnprojectionFailed(#[from] UnprojectionError),
}

pub struct Camera {
    // The focal length of the camera. This is the distance between the image plane and the center.
    pub focal_length: f64,
    // The resolution of the camera.
    pub width: u32,
    pub height: u32,
}

impl Camera {
    pub fn new(focal_length: f64, width: u32, height: u32) -> Self {
        Camera {
            focal_length,
            width,
            height,
        }
    }
}

/// The result of tracking an eye.
#[derive(Debug)]
pub struct TrackingResult {
    /// The timestamp of the observation.
    pub timestamp: f64,
    /// The sphere representing the eye in 3D space, from the camera's perspective.
    pub eye: Sphere3D,
    /// The circle representing the eye on the image plane.
    pub eye_projected: Circle2D,
    /// The circle representing the pupil in 3D space, from the camera's perspective.
    pub pupil: Circle3D,
    /// The ellipse representing the pupil on the image plane.
    pub pupil_projected: Option<Ellipse2D>,
    /// The confidence of the tracking result.
    pub confidence: f64,
}

const EYE_RADIUS_DEFAULT: f64 = 10.392304845413264;

pub struct EyeTracker3D {
    camera: Camera,
    short_term_model: EyeModel<FixedCache>,
    long_term_model: EyeModel<SpacialCache>,
    ultra_long_term_model: EyeModel<SpacialCache>,
    long_term_update_controller: TimedUpdateController,
    ultra_long_term_update_controller: TimedUpdateController,
}

impl EyeTracker3D {
    pub fn new(camera: Camera) -> Self {
        let short_term_model = EyeModel::new(FixedCache::new(10));
        let long_term_model = EyeModel::new(SpacialCache::new(
            camera.width as usize,
            camera.height as usize,
            10,
            30,
            300,
            5.0,
        ));
        let ultra_long_term_model = EyeModel::new(SpacialCache::new(
            camera.width as usize,
            camera.height as usize,
            10,
            30,
            600,
            60.0,
        ));
        let long_term_update_controller = TimedUpdateController::new(1.0, 5.0);
        let ultra_long_term_update_controller = TimedUpdateController::new(10.0, 5.0);
        EyeTracker3D {
            camera,
            short_term_model,
            long_term_model,
            ultra_long_term_model,
            long_term_update_controller,
            ultra_long_term_update_controller,
        }
    }

    /// Update the tracker with a new pupil ellipse, and return it's estimated gaze.
    /// The ellipse should be in the image's coordinate system, with the origin at the top left corner of the image.
    pub fn process(
        &mut self,
        timestamp: f64,
        ellipse: PupilEllipse,
    ) -> Result<TrackingResult, TrackingError> {
        // Convert the ellipse to the camera's coordinate system (0, 0 at the center of the image).
        let ellipse = Ellipse2D::new(
            ellipse.center.0 - self.camera.width as f64 / 2.0,
            ellipse.center.1 - self.camera.height as f64 / 2.0,
            // I'm not sure why these changes are needed, it should already be a radius, right?
            ellipse.major_radius / 2.0,
            ellipse.minor_radius / 2.0,
            // TODO: Somewhere something needs rotating, but I'm not sure where.
            ellipse.angle, // - std::f64::consts::PI / 2.0,
        );

        //----------------------------------------------------//
        // Step 1: Tell each model about the new observation. //
        //----------------------------------------------------//

        // Create a new observation and add it to each model.
        let observation = Rc::new(Observation::new(
            timestamp,
            ellipse,
            self.camera.focal_length,
        )?);
        self.short_term_model.add(observation.clone());
        self.long_term_model.add(observation.clone());
        self.ultra_long_term_model.add(observation.clone());

        //----------------------------------------------//
        // Step 2: Update the eye center of each model. //
        //----------------------------------------------//

        // Update the ultra long term eye center.
        if self
            .ultra_long_term_update_controller
            .should_update(timestamp)
        {
            self.ultra_long_term_model
                .update_eye_center(None, None, None)?;
        }

        // Update the long term eye center.
        if self.long_term_update_controller.should_update(timestamp) {
            self.long_term_model.update_eye_center(
                None,
                Some(self.ultra_long_term_model.unprojected_eye_center),
                Some(0.1),
            )?;
        }

        // Update the short term eye center.
        let circularity_mean = self.short_term_model.mean_observation_circularity();
        self.short_term_model.update_eye_center(
            Some(self.long_term_model.projected_eye_center),
            Some(self.long_term_model.unprojected_eye_center),
            Some(0.1 + 500.0 * 1.0 / (1.0 + (-(circularity_mean - 0.99) / 0.02).exp())),
        )?;

        //----------------------------------------------------------------//
        // Step 3: Estimate the pupil circle for the current observation. //
        //----------------------------------------------------------------//

        // Get the long term model's estimate of the pupil circle.
        let long_term_pupil_circle = self.long_term_model.get_pupil_circle(&observation);

        // Get the pupil normal.
        let normal = if self.long_term_update_controller.is_paused() {
            // If the long term model is paused, use it's estimate of the pupil normal.
            long_term_pupil_circle.normal
        } else {
            // Otherwise, use the short term model's estimate of the pupil normal.
            self.short_term_model.get_pupil_circle(&observation).normal
        };

        // Combine for the final estimate.
        let pupil_circle = Circle3D {
            center: long_term_pupil_circle.center,
            normal,
            radius: long_term_pupil_circle.radius,
        };

        //--------------------------------------//
        // Step 4: Apply refraction correction. //
        //--------------------------------------//

        let (corrected_sphere_center, corrected_pupil_circle) = self
            .short_term_model
            .apply_refraction_correction(&pupil_circle);

        //-----------------------------------//
        // Step 5: Return model information. //
        //-----------------------------------//

        // TODO: Calculate confidence.
        let confidence = 1.0;

        // Return the tracking result.
        Ok(TrackingResult {
            timestamp,
            eye: Sphere3D {
                center: corrected_sphere_center,
                radius: EYE_RADIUS_DEFAULT,
            },
            eye_projected: project_sphere_into_image_plane(
                &Sphere3D {
                    center: self.long_term_model.unprojected_eye_center,
                    radius: EYE_RADIUS_DEFAULT,
                },
                &self.camera,
            ),
            pupil: corrected_pupil_circle,
            pupil_projected: project_circle_into_image_plane(&pupil_circle, &self.camera),
            confidence,
        })
    }
}
