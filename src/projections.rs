//! Projections and unprojections of 2d and 3d objects.

use nalgebra::{Matrix3, Rotation3, RowVector3, Translation3, Vector2, Vector3};
use roots::{find_roots_cubic_normalized, Roots};
use thiserror::Error;

use crate::geometry::{Circle2D, Circle3D, Ellipse2D, Line2D, Sphere3D};
use crate::Camera;

#[derive(Error, Debug)]
pub enum UnprojectionError {
    #[error("No real roots")]
    NoRoots,
    #[error("Negative roots")]
    NegativeRoots,
}

/// Unprojects an ellipse from the 2d image, and returns the two possible
/// circles in 3d space for the given radius.
/// 
/// # Arguments
/// 
/// * `ellipse` - The ellipse to unproject.
/// * `focal_length` - The focal length of the camera.
/// * `radius` - The radius of the circle in 3d space.
/// 
/// # Returns
/// 
/// An array of two circles in 3d space.
/// 
/// # Errors
/// 
/// If the ellipse cannot be unprojected, an error is returned.
/// 
/// # References
/// 
/// * [Safaee-Rad, Tchoukanov, Smith, and Benhabib. 1992. Three-Dimensional Location Estimation of Circular Features for Machine Vision](https://www.eecg.toronto.edu/~pagiamt/kcsmith/00163786.pdf)
/// * [Świrski and Dodgson. 2013. A Fully-Automatic, Temporal Approach to Single Camera, Glint-Free 3D Eye Model Fitting.](http://www.cl.cam.ac.uk/research/rainbow/projects/eyemodelfit/) 
/// * https://github.com/LeszekSwirski/singleeyefitter/blob/master/lib/singleeyefitter/projection.h
/// * https://github.com/myirci/3d_circle_estimation/blob/master/algorithm/algorithm.cpp
/// 
#[allow(non_snake_case)]
pub fn unproject_ellipse(
    ellipse: &Ellipse2D,
    focal_length: f64,
    radius: f64,
) -> Result<[Circle3D; 2], UnprojectionError> {
    // Vertex of the cone in 3d space
    let alpha = 0.0;
    let beta = 0.0;
    let gamma = -focal_length;

    // Ellipse parameters
    let a = ellipse.major_radius;
    let b = ellipse.minor_radius;
    let x0 = ellipse.center.x;
    let y0 = ellipse.center.y;
    let θ = ellipse.angle;

    // Ellipse coefficients
    // https://en.wikipedia.org/wiki/Ellipse#General_ellipse
    //   Ax² + Bxy + Cy² + Dx + Ey + F = 0
    // where
    //   A = a²sin²θ + b²cos²θ
    //   B = 2(b² - a²)sinθcosθ
    //   C = a²cos²θ + b²sin²θ
    //   D = -2Ax₀ - By₀
    //   E = -Bx₀ - 2Cy₀
    //   F = Ax₀² + Bx₀y₀ + Cy₀² - a²b²
    let A = a * a * θ.sin() * θ.sin() + b * b * θ.cos() * θ.cos();
    let B = 2.0 * (b * b - a * a) * θ.sin() * θ.cos();
    let C = a * a * θ.cos() * θ.cos() + b * b * θ.sin() * θ.sin();
    let D = -2.0 * A * x0 - B * y0;
    let E = -B * x0 - 2.0 * C * y0;
    let F = A * x0 * x0 + B * x0 * y0 + C * y0 * y0 - a * a * b * b;

    // To the coefficients the paper wants - eq (1)
    // a'x² + 2h'xy + b'y² + 2g'x + 2f'y + d' = 0
    let a_prime = A;
    let h_prime = B / 2.0;
    let b_prime = C;
    let g_prime = D / 2.0;
    let f_prime = E / 2.0;
    let d_prime = F;

    // > Section III-A-3: The Computation Procedure for Surface-Normal Estimation

    // > Step 1 - Estimation of the coefficients of the general equation of the cone
    // > Estimate the coefficients a, b, c, f, g, h, u, v, w, and d as defined in (3)

    // (3) The general equation of the cone
    // >   ax² + by² + cz² + 2fyz + 2gzx + 2hxy + 2ux + 2vy + 2wz + d = 0
    // > where
    // >   a = γ²a'
    // >   b = γ²b'
    // >   c = a'α² + 2h'αβ + b'β² + 2g'α + 2f'β + d'
    // >   d = γ²d'
    // >   f = -γ(b'β + h'α + f')
    // >   g = -γ(h'β + a'α + g')
    // >   h = γ²h'
    // >   u = γ²g'
    // >   v = γ²f'
    // >   w = -γ(f'β + g'α + d')
    let a = gamma * gamma * a_prime;
    let b = gamma * gamma * b_prime;
    let c = a_prime * alpha * alpha
        + 2.0 * h_prime * alpha * beta
        + b_prime * beta * beta
        + 2.0 * g_prime * alpha
        + 2.0 * f_prime * beta
        + d_prime;
    // let d = gamma * gamma * d_prime; // Unused? Did I miss something?
    let f = -gamma * (b_prime * beta + h_prime * alpha + f_prime);
    let g = -gamma * (h_prime * beta + a_prime * alpha + g_prime);
    let h = gamma * gamma * h_prime;
    let u = gamma * gamma * g_prime;
    let v = gamma * gamma * f_prime;
    let w = -gamma * (f_prime * beta + g_prime * alpha + d_prime);

    // > Step 2 - Reduction of the equation of the cone
    // > Determine the coefficients λ₁, λ₂, and λ₃ in (18), by solving the
    // > discriminating cubic equation (10), such that λ₁ and λ₂ are positive.

    // (10) Discriminating cubic equation
    // > λ³-λ²(a+b+c)+λ(bc+ca+ab-f²-g²-h²)-(abc+2fgh-af²-bg²-ch²) = 0
    // TODO: check performance of this
    // TODO: "such that λ₁ and λ₂ are positive." - what if they aren't?
    // Note: this function returns roots in increasing order, we'll want
    // them decreasing to make the remaining process easier
    let λ = match find_roots_cubic_normalized(
        -(a + b + c),
        b * c + c * a + a * b - f * f - g * g - h * h,
        -(a * b * c + 2.0 * f * g * h - a * f * f - b * g * g - c * h * h),
    ) {
        Roots::No(_) => return Err(UnprojectionError::NoRoots),
        Roots::One([root]) => Vector3::from_element(root),
        Roots::Three([root1, root2, root3]) => Vector3::new(root3, root2, root1),
        _ => panic!("Unexpected number of roots"),
    };
    // assert!(λ[0] >= λ[1]);
    // assert!(λ[0] >= 0.0);
    // assert!(λ[1] >= 0.0);
    if λ[0] < 0.0 || λ[1] < 0.0 {
        return Err(UnprojectionError::NegativeRoots);
    }


    // > Step 3 - Estimation of the coefficients of the equation of the circular-feature plane
    // > Having estimated the coefficients of the central cone in step 2 (λᵢ in (18)),
    // > three possible cases occur:
    // >   1) λ₁ < λ₂, for which the solutions would be (30)
    // >   2) λ₁ > λ₂, for which the solutions would be (31)
    // >   3) λ₁ = λ₂, for which the solutions would be (33)

    // The paper doesn't specify the order of the roots, so we'll assume it doesn't matter
    // Since they are in decending order, we only need solution 2)
    // (unless λ₁ = λ₂? I don't know if that can happen.)

    let n = ((λ[1] - λ[2]) / (λ[0] - λ[2])).sqrt();
    let m = 0.0;
    let l = ((λ[0] - λ[1]) / (λ[0] - λ[2])).sqrt(); // + and -

    // > Step 4 - Estimation of the direction cosines of the surface normal with
    // > respect to the camera frame

    // > First, estimate the elements of the rotational transformation between
    // > the z’y’z’ frame and the zyz frame (see (8)) by using (12)

    // (12) Elements of the rotational transformation
    // >   mᵢ = 1/sqrt(1+(t₁/t₂)²+t₃²)
    // >   lᵢ = (t₁/t₂)mᵢ
    // >   nᵢ = t₃mᵢ
    // > where
    // >   t₁ = (b-λᵢ)g-fh
    // >   t₂ = (a-λᵢ)f-gh
    // >   t₃ = -(a-λᵢ)(t₁/t₂)/g - h/g
    let t1 = Vector3::new(
        (b - λ[0]) * g - f * h,
        (b - λ[1]) * g - f * h,
        (b - λ[2]) * g - f * h,
    );
    let t2 = Vector3::new(
        (a - λ[0]) * f - g * h,
        (a - λ[1]) * f - g * h,
        (a - λ[2]) * f - g * h,
    );
    let t3 = Vector3::new(
        -(a - λ[0]) * (t1[0] / t2[0]) / g - h / g,
        -(a - λ[1]) * (t1[1] / t2[1]) / g - h / g,
        -(a - λ[2]) * (t1[2] / t2[2]) / g - h / g,
    );
    let mut mi = RowVector3::new(
        1.0 / (1.0 + (t1[0] / t2[0]) * (t1[0] / t2[0]) + t3[0] * t3[0]).sqrt(),
        1.0 / (1.0 + (t1[1] / t2[1]) * (t1[1] / t2[1]) + t3[1] * t3[1]).sqrt(),
        1.0 / (1.0 + (t1[2] / t2[2]) * (t1[2] / t2[2]) + t3[2] * t3[2]).sqrt(),
    );
    let mut li = RowVector3::new(
        (t1[0] / t2[0]) * mi[0],
        (t1[1] / t2[1]) * mi[1],
        (t1[2] / t2[2]) * mi[2],
    );
    let mut ni = RowVector3::new(t3[0] * mi[0], t3[1] * mi[1], t3[2] * mi[2]);

    // "The estimated values for lᵢ, mᵢ, and nᵢ must satisfy the right-hand rule."
    // I don't know what this means lol
    // https://math.stackexchange.com/a/4274758
    // > Given that v1, v2 and v3 are defined in a right-handed coordinate
    // > system, if (v1×v2)⋅v3 > 0 then it's right-handed, while if it's less
    // > than 0, it's left handed.
    // https://en.wikipedia.org/wiki/Right-hand_rule
    // > Reversing the direction of one axis (or of all three axes) also
    // > reverses the handedness.
    if li.cross(&mi).dot(&ni) < 0.0 {
        mi.neg_mut();
        li.neg_mut();
        ni.neg_mut();
    }

    // Finally have T₁
    let T1 = Matrix3::<f64>::from_rows(&[li, mi, ni]);
    let T1 = Rotation3::from_matrix_unchecked(T1);

    // Skipping ahead a bit to precompute T₂ since it doesn't rely on which solution we use
    // > To reduce (13) to (6), the following translational transformation (T₂) is applied
    let T2 = Translation3::new(
        -(u * li[0] + v * mi[0] + w * ni[0]) / λ[0],
        -(u * li[1] + v * mi[1] + w * ni[1]) / λ[1],
        -(u * li[2] + v * mi[2] + w * ni[2]) / λ[2],
    );

    // The remaining stuff depends on which solution from Step 3 we use.
    // We don't know which one we need, so we'll do both.
    let mut circles: [Circle3D; 2] = Default::default();
    for i in 0..2 {
        // Since we used case 2 in Step 3, the l value has two solutions (+l and -l)
        let l = if i == 0 { l } else { -l };

        // > Then the coefficients of the equation of the desired plane with respect
        // > to the camera zyz frame can be estimated by applying this rotational
        // > transformation.

        // This is our surface normal I think? See below.
        let mut normal = T1 * Vector3::new(l, m, n);

        // > Following which, the direction cosines of the surface normal are
        // > estimated using (5).
        // Not sure what this means, there's no mention of cosines in (5).
        // > Having found the coefficients of the equation of the plane, the
        // > direction numbers (l’, m’, n’) of the orientation of the circular
        // > feature can be estimated from l/l’ = m/m’ = n/n’
        // So, yea, idk, the paper is unclear, and their example (Section V-A-1)
        // doesn't do anything here either.

        // Pupil center time

        // The paper doesn't have a computation procedure for the center, so we have
        // to figure it out ourselves from the example.

        // > For 3-D position estimation, the estimation process is as follows:

        // > 5) Based on the estimated parameters in step 3, the rotational
        // >    transformation (36) is obtained

        // (36) doesn't look right, but it points to (19) I think?
        // > The elements of the transformation (19) are already known (since the coefficients
        // > of the equation of the desired plane l, m, n are known through application of the
        // > above-mentioned computation procedure in Section 111-A-3)
        // Which is called T₃ so that matches what we're looking for...
        // > In order to find the equation of the intersection curve of the above two surfaces,
        // > the following rotational transformation (T₃) can be used.
        // (Not gonna transcribe the matrix, it's in the paper)

        // TODO: this can be simplified since we only used one of the cases in
        // Step 3 - We're using case 2) which means that m = 0.0

        let T3_0_0 = -m / (l * l + m * m).sqrt();
        let T3_0_1 = -((l * n) / (l * l + m * m).sqrt());
        let T3_0_2 = l;
        let T3_1_0 = l / (l * l + m * m).sqrt();
        let T3_1_1 = -((m * n) / (l * l + m * m).sqrt());
        let T3_1_2 = m;
        let T3_2_0 = 0.0;
        let T3_2_1 = (l * l + m * m).sqrt();
        let T3_2_2 = n;

        let T3 = Matrix3::new(
            T3_0_0, T3_0_1, T3_0_2, T3_1_0, T3_1_1, T3_1_2, T3_2_0, T3_2_1, T3_2_2,
        );

        let T3_l = T3.fixed_view::<3, 1>(0, 0);
        let T3_m = T3.fixed_view::<3, 1>(0, 1);
        let T3_n = T3.fixed_view::<3, 1>(0, 2);

        // Needs to be an actual rotation for later
        let T3 = Rotation3::from_matrix_unchecked(T3);

        // > 6) Parameters A, B, C, and D are estimated using the elements of
        // >    the above transformation (T₃) based on (38)

        // (38)
        // > A ≡ (λ₁l₁² + λ₂l₂² + λ₃l₃²)
        // > B ≡ (λ₁l₁n₁ + λ₂l₂n₂ + λ₃l₃n₃)
        // > C ≡ (λ₁m₁n₁ + λ₂m₂n₂ + λ₃m₃n₃)
        // > D ≡ (λ₁n₁² + λ₂n₂² + λ₃n₃²)

        // These l/m/n are from the T₃ matrix above, not the l/m/n from
        // Step 3 or the li/mi/ni from Step 4

        let A = λ[0] * T3_l[0] * T3_l[0] + λ[1] * T3_l[1] * T3_l[1] + λ[2] * T3_l[2] * T3_l[2];
        let B = λ[0] * T3_l[0] * T3_n[0] + λ[1] * T3_l[1] * T3_n[1] + λ[2] * T3_l[2] * T3_n[2];
        let C = λ[0] * T3_m[0] * T3_n[0] + λ[1] * T3_m[1] * T3_n[1] + λ[2] * T3_m[2] * T3_n[2];
        let D = λ[0] * T3_n[0] * T3_n[0] + λ[1] * T3_n[1] * T3_n[1] + λ[2] * T3_n[2] * T3_n[2];

        // > 7) The 3-D position coordinates with respect to the X'Y'Z' frame
        // >    are estimated using (41)
        // looking back in the paper:
        // > the coordinates of the center of the circle with respect to the X’Y’Z’ frame are
        // > (41)
        // > under the condition that the sign of the coordinate Z' is selected
        // > such that the coordinate z, in the xyz frame would be positive.
        // No clue what that second part means for us, but because of math things
        // we can just flip all the signs later if needed
        let Z_prime = A * radius / (B * B + C * C - A * D).sqrt();
        let X_prime = -(B / A) * Z_prime;
        let Y_prime = -(C / A) * Z_prime;

        let mut center_prime = Vector3::new(X_prime, Y_prime, Z_prime);

        // > 8) To obtain the 3-D position of the feature with respect to the camera
        // >    frame, we must apply the total transformation as defined in (35).

        // > First, the translational transformation (14), T2, is estimated using
        // > the parameter values obtained in steps 1, 2, and 4

        // (We already did this outside the loop)

        // > The transformation between the image frame and the camera frame is (...)
        // The example shows T₀, so this is (34)
        // TODO: can be done outside the loop
        let T0 = Translation3::new(0.0, 0.0, focal_length);

        // Then we can do the total transformation (35)
        let T = T0 * T1 * T2 * T3;
        let mut center = T * center_prime;

        // > Note that we take the negative sign for Z' in order to get the
        // > positive value for z in the camera frame.
        // Positive z means in front of the camera, which is what we want, so
        // if it's negative, flip Z' and try again
        if center[2] < 0.0 {
            center_prime.neg_mut();
            center = T * center_prime;
        }

        // Clean up the normal and make sure it faces the camera
        // [Swirski and Dodgson 2013]
        // > If necessary, the gaze is ﬂipped so that it points ‘towards’ the camera.
        if normal.dot(&center) > 0.0 {
            normal = -normal;
        }
        normal.normalize_mut();

        // That's ...it I think? We have a normal and the center point.
        circles[i] = Circle3D {
            center,
            normal,
            radius,
        };
    }

    Ok(circles)
}

/// Projects a 3D point into the image plane. Assumes the camera is
/// at `(0, 0, 0)` and the image plane is at `z = focal_length`.
/// 
/// # Arguments
/// 
/// * `point` - The 3D point to project
/// * `focal_length` - The focal length of the camera
/// 
/// # Returns
/// 
/// * The 2D point in the image plane
pub fn project_point_into_image_plane(
    point: nalgebra::Vector3<f64>,
    focal_length: f64,
) -> nalgebra::Vector2<f64> {
    let scale = focal_length / point[2];
    let point_projected = scale * point;
    point_projected.fixed_view::<2, 1>(0, 0).into_owned()
}

/// Projects a 3D line into the image plane. Assumes the camera is
/// at `(0, 0, 0)` and the image plane is at `z = focal_length`.
/// 
/// # Arguments
/// 
/// * `origin` - The origin of the 3D line
/// * `direction` - The direction of the 3D line
/// * `focal_length` - The focal length of the camera
/// 
/// # Returns
/// 
/// * The 2D line in the image plane
pub fn project_line_into_image_plane(origin: Vector3<f64>, direction: Vector3<f64>, focal_length: f64) -> Line2D {
    // Get two points on the line
    let p1 = origin;
    let p2 = origin + direction;

    // Project them into the image plane
    let p1_projected: nalgebra::Vector2<f64> = project_point_into_image_plane(p1, focal_length);
    let p2_projected: nalgebra::Vector2<f64> = project_point_into_image_plane(p2, focal_length);

    // Return the line between them
    Line2D {
        origin: p1_projected,
        direction: p2_projected - p1_projected,
    }
}

/// Projects a 3D circle into the image plane. Assumes the camera is
/// at `(0, 0, 0)` and the image plane is at `z = focal_length`. The
/// projected ellipse is returned using image coordinates (i.e. the
/// origin is at the top left of the image).
/// 
/// # Arguments
/// 
/// * `circle` - The 3D circle to project
/// * `focal_length` - The focal length of the camera
/// 
/// # Returns
/// 
/// * The 2D ellipse in the image plane
/// 
#[allow(non_snake_case)]
pub fn project_circle_into_image_plane(circle: &Circle3D, camera: &Camera) -> Option<Ellipse2D> {
    let c = circle.center;
    let n = circle.normal;
    let r = circle.radius;
    let f = camera.focal_length;

    let cn = c.dot(&n);
    let c2r2 = c.dot(&c) - r.powi(2);
    let ABC_0 = cn.powi(2);
    let ABC_1 = 2.0 * cn * c.component_mul(&n);
    let ABC_2 = c2r2 * n.map(|v| v.powi(2));
    let ABC = ABC_1.map(|v| ABC_0 - v) + ABC_2;

    let F = 2.0 * (c2r2 * n[1] * n[2] - cn * (n[1] * c[2] + n[2] * c[1]));
    let G = 2.0 * (c2r2 * n[2] * n[0] - cn * (n[2] * c[0] + n[0] * c[2]));
    let H = 2.0 * (c2r2 * n[0] * n[1] - cn * (n[0] * c[1] + n[1] * c[0]));

    let A = ABC[0];
    let B = H;
    let C = ABC[1];
    let D = G * f;
    let E = F * f;
    let F = ABC[2] * f.powi(2);

    let disc_ = B.powi(2) - 4.0 * A * C;

    if disc_ < 0.0 {
        let center_x = (2.0 * C * D - B * E) / disc_;
        let center_y = (2.0 * A * E - B * D) / disc_;
        let temp_ = 2.0 * (A * E.powi(2) + C * D.powi(2) - B * D * E + disc_ * F);
        let minor_axis =
            -((temp_ * (A + C - ((A - C).powi(2) + B.powi(2)).sqrt())).abs()).sqrt() / disc_;
        let major_axis =
            -((temp_ * (A + C + ((A - C).powi(2) + B.powi(2)).sqrt())).abs()).sqrt() / disc_;

        let angle = if B == 0.0 {
            if A < C {
                0.0
            } else {
                std::f64::consts::PI / 2.0
            }
        } else {
            ((C - A - ((A - C).powi(2) + B.powi(2)).sqrt()) / B).atan()
        };

        Some(Ellipse2D {
            center: Vector2::new(
                center_x + camera.width as f64 / 2.0,
                center_y + camera.height as f64 / 2.0,
            ),
            minor_radius: 2.0 * minor_axis,
            major_radius: 2.0 * major_axis,
            angle: angle * 180.0 / std::f64::consts::PI + 90.0,
        })
    } else {
        None
    }
}

/// Projects a 3D sphere into the image plane. Assumes the camera is
/// at `(0, 0, 0)` and the image plane is at `z = focal_length`. The
/// projected circle is returned using image coordinates (i.e. the
/// origin is at the top left corner of the image).
/// 
/// # Arguments
/// 
/// * `sphere` - The 3D sphere to project
/// * `focal_length` - The focal length of the camera
/// 
/// # Returns
/// 
/// * The 2D circle in the image plane.
pub fn project_sphere_into_image_plane(sphere: &Sphere3D, camera: &Camera) -> Circle2D {
    let scale = camera.focal_length / sphere.center.z;

    let mut projected_sphere_center = sphere.center.scale(scale);
    let mut projected_radius = scale * sphere.radius;

    projected_sphere_center[0] += camera.width as f64 / 2.0;
    projected_sphere_center[1] += camera.height as f64 / 2.0;
    projected_radius *= 2.0;

    Circle2D {
        center: projected_sphere_center
            .fixed_view::<2, 1>(0, 0)
            .clone_owned(),
        radius: projected_radius,
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_util::*;
    use std::error::Error;

    test_data! {
        "../test_data/projections.json" as
        unproject_ellipse: (Ellipse2D, f64, f64) => [Circle3D; 2],
    }

    #[test]
    fn test_unproject_ellipse() -> Result<(), Box<dyn Error>> {
        for (input, output) in &TEST_DATA.unproject_ellipse {
            assert_float_eq!(unproject_ellipse(&input.0, input.1, input.2)?, output);
        }
        Ok(())
    }

    // TODO: add projection tests
}
