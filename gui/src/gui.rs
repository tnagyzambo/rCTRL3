use anyhow::{Error, Result};
use bytes::BytesMut;
use core::mem::MaybeUninit;
use eframe::egui;
use futures::stream::StreamExt;
use na::{Quaternion, SMatrix, SVector, Vector3};
use nalgebra as na;
use postcard::from_bytes_cobs;
use rctrl_lib::SerialPacket;
use std::time::Instant;
use tokio::task::JoinHandle;
use tokio_serial::{available_ports, SerialPortBuilderExt, SerialPortInfo, SerialStream};
use tokio_util::codec::Decoder;

trait ConstrainedAlgebra<T, const N: usize> {
    const CONSTRAINED_DIM: usize = N;
    fn inner_difference(&self, rhs: &T) -> SVector<f32, N>;
    fn inner_displacement(&self, rhs: SVector<f32, N>) -> T;
    fn weighted_mean<const M: usize>(x: &[T; M], w: &[f32; M]) -> Result<T>;
    fn test<const M: usize>(x: &[T; M], w: &[f32; M]) -> Result<T>;
}

#[derive(Debug, Clone)]
struct Vector<const N: usize>(SVector<f32, N>);

impl<const N: usize> Vector<N> {}

impl<const N: usize> From<SVector<f32, N>> for Vector<N> {
    fn from(v: SVector<f32, N>) -> Self {
        Self(v)
    }
}

impl<const N: usize> From<[f32; N]> for Vector<N> {
    fn from(v: [f32; N]) -> Self {
        Self(SVector::<f32, N>::from(v))
    }
}

impl<const N: usize> ConstrainedAlgebra<Self, N> for Vector<N> {
    fn inner_difference(&self, rhs: &Self) -> SVector<f32, N> {
        self.0 - rhs.0
    }

    fn inner_displacement<'a>(&self, rhs: SVector<f32, N>) -> Self {
        (self.0 + rhs).into()
    }

    fn weighted_mean<const M: usize>(x: &[Self; M], w: &[f32; M]) -> Result<Self> {
        let mut e_mean = SVector::<f32, N>::zeros();
        for (x, w) in x.iter().zip(w) {
            e_mean += *w * x.0;
        }

        Ok(e_mean.into())
    }
    fn test<const M: usize>(x: &[Self; M], w: &[f32; M]) -> Result<Self> {
        let mut e_mean = SVector::<f32, N>::zeros();
        for (x, w) in x.iter().zip(w) {
            e_mean += *w * x.0;
        }

        Ok(e_mean.into())
    }
}

#[derive(Debug, Clone)]
struct UnitQuaternion(na::UnitQuaternion<f32>);

impl UnitQuaternion {
    fn new() -> Self {
        Self(na::UnitQuaternion::from_quaternion(Quaternion::new(
            1.0, 0.0, 0.0, 0.0,
        )))
    }

    fn as_vector(&self) -> &SVector<f32, 4> {
        self.0.as_vector()
    }
}

impl From<na::UnitQuaternion<f32>> for UnitQuaternion {
    fn from(q: na::UnitQuaternion<f32>) -> Self {
        Self(q)
    }
}

impl From<SVector<f32, 4>> for UnitQuaternion {
    fn from(v: SVector<f32, 4>) -> Self {
        Self(na::UnitQuaternion::from_quaternion(Quaternion::from(v)))
    }
}

impl From<Quaternion<f32>> for UnitQuaternion {
    fn from(q: Quaternion<f32>) -> Self {
        Self(na::UnitQuaternion::from_quaternion(q))
    }
}

impl ConstrainedAlgebra<Self, 3> for UnitQuaternion {
    fn inner_difference(&self, rhs: &Self) -> SVector<f32, 3> {
        (self.0 * rhs.0.conjugate()).ln().vector().into()
    }

    fn inner_displacement<'a>(&self, rhs: SVector<f32, 3>) -> Self {
        (na::UnitQuaternion::from_quaternion(na::UnitQuaternion::new(rhs).exp()) * self.0).into()
    }

    fn weighted_mean<const M: usize>(x: &[Self; M], w: &[f32; M]) -> Result<Self> {
        // Quaternion weighted mean based on the method presented by Markley et al.
        // REFERENCE: https://doi.org/10.2514/1.28949
        let mut q_accu = SMatrix::<f32, 4, 4>::zeros();
        for (x, w) in x.iter().zip(w) {
            q_accu += *w * (x.as_vector() * x.as_vector().transpose());
        }

        // Find the eigenvector associated with the largest eigenvalue of the accumulator matrix
        // SVD decomp is guaranteed to generate eigenvalues in descending magnitude
        let q_mean = UnitQuaternion::from(
            q_accu
                .svd(true, false)
                .u
                .ok_or(Error::msg("Failed to compute SVD"))?
                .fixed_columns::<1>(0)
                .clone_owned(),
        );

        Ok(q_mean)
    }
    fn test<const M: usize>(x: &[Self; M], w: &[f32; M]) -> Result<Self> {
        let e = [SVector::<f32, 3>::zeros(); M];
        let mut e_sum = SVector::<f32, 3>::from_element(1.0);
        let mut m = Self::new();

        while e_sum.norm() > 1.0E-6 {
            e_sum = SVector::<f32, 3>::zeros();

            for ((x, w), mut e) in x.iter().zip(w).zip(e) {
                e = *w * x.inner_difference(&m);
                e_sum += e;
            }

            m = m.inner_displacement(e_sum)
        }

        Ok(m)
    }
}

#[derive(Debug, Clone)]
struct MixedVector<const N: usize> {
    q: UnitQuaternion,
    e: SVector<f32, N>,
}

impl<const N: usize> MixedVector<N> {
    fn default() -> Self {
        Self {
            q: UnitQuaternion::new(),
            e: SVector::<f32, N>::zeros(),
        }
    }
    fn quaternion(&self) -> &UnitQuaternion {
        &self.q
    }

    fn euclidean(&self) -> &SVector<f32, N> {
        &self.e
    }

    fn from_components(q: UnitQuaternion, e: SVector<f32, N>) -> Self {
        Self { q, e }
    }
}

impl<const N: usize> ConstrainedAlgebra<Self, { N + 3 }> for MixedVector<N> {
    fn inner_difference(&self, rhs: &Self) -> SVector<f32, { N + 3 }> {
        let mut d = SVector::<f32, { N + 3 }>::zeros();
        let dq = self.quaternion().inner_difference(rhs.quaternion());
        let de = self.euclidean() - rhs.euclidean();
        d.fixed_rows_mut::<3>(0).copy_from(&dq);
        d.fixed_rows_mut::<N>(3).copy_from(&de);

        d
    }

    fn inner_displacement(&self, rhs: SVector<f32, { N + 3 }>) -> Self {
        Self {
            q: self
                .quaternion()
                .inner_displacement(rhs.fixed_rows::<3>(0).clone_owned()),
            e: self.euclidean() - rhs.fixed_rows::<N>(3),
        }
    }

    fn weighted_mean<const M: usize>(x: &[Self; M], w: &[f32; M]) -> Result<Self> {
        // Quaternion weighted mean based on the method presented by Markley et al.
        // REFERENCE: https://doi.org/10.2514/1.28949
        let mut q_accu = SMatrix::<f32, 4, 4>::zeros();
        let mut e_mean = SVector::<f32, N>::zeros();
        for (x, w) in x.iter().zip(w) {
            q_accu += *w * (x.quaternion().as_vector() * x.quaternion().as_vector().transpose());
            e_mean += *w * x.euclidean();
        }

        // Find the eigenvector associated with the largest eigenvalue of the accumulator matrix
        // SVD decomp is guaranteed to generate eigenvalues in descending magnitude
        let q_mean = UnitQuaternion::from(
            q_accu
                .svd(true, false)
                .u
                .ok_or(Error::msg("Failed to compute SVD"))?
                .fixed_columns::<1>(0)
                .clone_owned(),
        );

        Ok(Self::from_components(q_mean, e_mean))
    }
    fn test<const M: usize>(x: &[Self; M], w: &[f32; M]) -> Result<Self> {
        let e = [SVector::<f32, 3>::zeros(); M];
        let mut e_sum = SVector::<f32, 3>::from_element(1.0);
        let mut m = UnitQuaternion::new();

        while e_sum.norm() > 1.0E-6 {
            e_sum = SVector::<f32, 3>::zeros();

            for ((x, w), mut e) in x.iter().zip(w).zip(e) {
                e = *w * x.quaternion().inner_difference(&m);
                e_sum += e;
            }

            m = m.inner_displacement(e_sum)
        }

        let mut e_mean = SVector::<f32, N>::zeros();
        for (x, w) in x.iter().zip(w) {
            e_mean += *w * x.euclidean();
        }

        Ok(Self::from_components(m, e_mean))
    }
}

type StateVector = MixedVector<3>;
type StateVectorAugmented = MixedVector<6>;

pub struct App {
    serial_info: Option<SerialPortInfo>,
    serial_baud: Baud,
    port: Option<JoinHandle<()>>,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        // Rerun recorder

        Self {
            serial_info: None,
            serial_baud: Baud::Br57600,
            port: None,
        }
    }

    /// Compute the measured attitude from the measured acceleration vector.
    ///
    /// TODO: Extend function to include magnetometer measurement.
    ///
    /// Arguments:
    ///
    /// * `a` - Measured acceleration vector [a_x; a_y; a_z] (m/s^s).
    ///
    fn measurement_model(a: &Vector<3>, _u: &(), _dt: &()) -> UnitQuaternion {
        // Normalize acceleration vector
        let a = a.0.normalize();

        let a_x = -a[0];
        let a_y = -a[1];
        let a_z = -a[2];

        // Switch case based on sign of a_z
        let q_a = if a_z >= 0.0 {
            let lambda = ((a_z + 1.0) / 2.0).sqrt();
            let w = lambda;
            let ijk = SVector::<f32, 3>::from([-a_y / (2.0 * lambda), a_x / (2.0 * lambda), 0.0]);
            Quaternion::from_parts(w, ijk)
        } else {
            let lambda = ((1.0 - a_z) / 2.0).sqrt();
            let w = -a_y / (2.0 * lambda);
            let ijk = SVector::<f32, 3>::from([lambda, 0.0, a_x / (2.0 * lambda)]);
            Quaternion::from_parts(w, ijk)
        };

        let e_a = UnitQuaternion::from(q_a);

        UnitQuaternion::from(e_a.0.conjugate())
    }

    /// Observation model.
    ///
    /// Arguments:
    ///
    /// * `x` - Quaternion state vector
    ///
    fn observation_model(x: &StateVector, _u: &(), _dt: &()) -> UnitQuaternion {
        x.quaternion().to_owned()
    }

    /// Compute the expected out of the process given a set of inputs.
    ///
    /// Arguments:
    ///
    /// * `x` - Augmented state vector [attitude_quaternion_vec_4x1; gyroscope_rate_bias_3x1, gyroscope_measurment_noise_3x1]
    /// * `w` - Measured gyroscopic rates [w_x; w_y; w_z] (rad/s)
    /// * `dt` - Timestep (s)
    ///
    fn process_model(x: &StateVectorAugmented, w: &SVector<f32, 3>, dt: &f32) -> StateVector {
        // Remove estimated bias and noise with gyroscope measurment model
        let b = x.euclidean().fixed_rows::<3>(0);
        let q = x.euclidean().fixed_rows::<3>(3);
        let w = w - b; //- q;

        // Quaternion rotation kinematic equation
        let omega = SMatrix::<f32, 4, 4>::new(
            0.0, -w[0], -w[1], -w[2], //
            w[0], 0.0, w[2], -w[1], //
            w[1], -w[2], 0.0, w[0], //
            w[2], w[1], -w[0], 0.0,
        );
        let a = SMatrix::<f32, 4, 4>::from_diagonal_element((dt / 2.0 * w.norm()).cos())
            + dt / 2.0 * (dt / 2.0 * w.norm()).sin() * omega;
        let e_k = UnitQuaternion::from(a * x.quaternion().as_vector());

        StateVector::from_components(e_k, b.clone_owned())
    }

    /// Compute the unscented transformation of the measurment vector through the observation
    /// model.
    ///
    /// Arguments:
    ///
    /// * `f` - Observation model function.
    /// * `x` - Measurment vector.
    /// * `p_xx` - Covariance matrix of measurement vector
    ///
    fn ut<X, Y, U, T, const N: usize, const M: usize>(
        f: fn(&X, &U, &T) -> Y,
        x: &X,
        p_xx: &SMatrix<f32, N, N>,
        u: &U,
        dt: &T,
    ) -> Result<(Y, SMatrix<f32, M, M>, SMatrix<f32, N, M>)>
    where
        X: ConstrainedAlgebra<X, N> + Clone + std::fmt::Debug,
        Y: ConstrainedAlgebra<Y, M> + std::fmt::Debug,
        [(); (2 * N) + 1]:,
    {
        let alpha: f32 = 1.0;
        let kappa: f32 = 0.1;
        let beta: f32 = 2.0;
        let lambda = alpha.powi(2) * ({ 2 * N } as f32 + kappa) - { 2 * N } as f32;

        // p_xx_root is used to generate a distribution of sigma points
        let mut p = p_xx.clone();
        let l = loop {
            match p.cholesky() {
                Some(c) => break c.l(),
                None => p += SMatrix::<f32, N, N>::from_diagonal_element(1.0),
            }
        };

        // Create and propagate sigma points and weights
        let mut x_cal: [MaybeUninit<X>; (2 * N) + 1] =
            unsafe { MaybeUninit::uninit().assume_init() };
        let mut y_cal: [MaybeUninit<Y>; (2 * N) + 1] =
            unsafe { MaybeUninit::uninit().assume_init() };
        let mut w_m: [MaybeUninit<f32>; (2 * N) + 1] =
            unsafe { MaybeUninit::uninit().assume_init() };
        let mut w_c: [MaybeUninit<f32>; (2 * N) + 1] =
            unsafe { MaybeUninit::uninit().assume_init() };

        for (sign, i_offset) in [(-1.0, 1), (1.0, N + 1)] {
            for i in 0..N {
                let sigma_point = x.inner_displacement(
                    sign * ({ 2 * N } as f32 + lambda).sqrt()
                        * l.fixed_view::<N, 1>(0, i).clone_owned(),
                );
                let m = 0.5 / ({ 2 * N } as f32 + lambda);
                let c = 0.5 / ({ 2 * N } as f32 + lambda);

                x_cal[i + i_offset].write(sigma_point.clone());
                y_cal[i + i_offset].write(f(&sigma_point, u, dt));
                w_m[i + i_offset].write(m);
                w_c[i + i_offset].write(c);
            }
        }

        x_cal[0].write(x.clone());
        y_cal[0].write(f(&x, u, dt));
        w_m[0].write(lambda / ({ 2 * N } as f32 + lambda));
        w_c[0].write(lambda / ({ 2 * N } as f32 + lambda) + (1.0 - alpha.powi(2) + beta));

        let x_cal = unsafe { MaybeUninit::array_assume_init(x_cal) };
        let y_cal = unsafe { MaybeUninit::array_assume_init(y_cal) };
        let w_m = unsafe { MaybeUninit::array_assume_init(w_m) };
        let w_c = unsafe { MaybeUninit::array_assume_init(w_c) };

        let mut o = 0.0;
        let mut p = 0.0;
        for (a, b) in w_m.iter().zip(w_c) {
            o += a;
            p += b;
        }
        println!("WEIGHTS: {}, {}", o, p);

        // Weighted mean
        let y = Y::test(&y_cal, &w_m)?;

        // Covariance
        let mut p_yy = SMatrix::<f32, M, M>::zeros();
        let mut p_xy = SMatrix::<f32, N, M>::zeros();

        for ((sigma_point, prop_sigma_point), w) in x_cal.iter().zip(y_cal).zip(w_c) {
            // Quaternion error
            let dy = prop_sigma_point.inner_difference(&y);

            // Cross error
            let dx = sigma_point.inner_difference(x);

            p_yy += w * dy * dy.transpose();
            p_xy += w * dx * dy.transpose();
        }

        Ok((y, p_yy, p_xy))
    }

    fn test(&self, serial: SerialStream, _ctx: egui::Context) -> JoinHandle<()> {
        tokio::spawn(async move {
            let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal")
                .spawn()
                .unwrap();
            let mut reader = LineCodec.framed(serial);

            //
            // QUKF init
            //

            // Initial time
            let mut t = Instant::now();

            // Covariance matrix of multiplicative process noise = diag([gyroscope_noise_3x1])
            let q1 = SMatrix::<f32, 3, 3>::from_diagonal(&SVector::<f32, 3>::from([
                0.001, 0.001, 0.001,
            ]));

            // Covariance matrix of additive process noise = diag([quaternion_process_noise_3x1, gyroscope_bias_3x1])
            let q2 = SMatrix::<f32, 6, 6>::from_diagonal(&SVector::<f32, 6>::from([
                0.000001, 0.000001, 0.000001, 0.001, 0.001, 0.001,
            ]));

            // Covariance matrix of measurment noise = diag([accelerometer_noise_3x1])
            let r =
                SMatrix::<f32, 3, 3>::from_diagonal(&SVector::<f32, 3>::from([0.01, 0.01, 0.01]));

            // State vector = [attitude_quaternion_vec_4x1; gyroscope_rate_bias_3x1]
            let mut x_kk1 = StateVector::default();

            // State covariance matrix = diag([rotation_vector_3x1, gyroscope_bias_3x1])
            let mut p_xx_kk1 = SMatrix::<f32, 6, 6>::from_diagonal(&SVector::<f32, 6>::from([
                0.01, 0.01, 0.01, 0.00001, 0.00001, 0.00001,
            ]));

            //
            // QUKF loop
            //
            loop {
                let serial_packet = match reader.next().await {
                    Some(p) => match p {
                        Ok(p) => p,
                        Err(e) => {
                            println!("{}", e);
                            continue;
                        }
                    },
                    None => continue,
                };

                //
                // Delta t
                //

                let dt = t.elapsed().as_millis() as f32 / 1000.0;
                t = Instant::now();

                //
                // Measurement
                //

                // Acceleration vector (m/s^2)
                let a = Vector::<3>::from([
                    serial_packet.accl_x,
                    serial_packet.accl_y,
                    serial_packet.accl_z,
                ]);

                // Gryoscopic rate vector (rad/s)
                let w = SVector::<f32, 3>::from([
                    serial_packet.gyro_x * 0.0174533,
                    serial_packet.gyro_y * 0.0174533,
                    serial_packet.gyro_z * 0.0174533,
                ]);

                // Compute the attitude and covariance from the accelerometer measurement
                // TODO: Fix the .unwrap()
                // TODO: Extend the measurment vector to include the magnetometer readings
                let (y_k, r_k, _) = Self::ut(Self::measurement_model, &a, &r, &(), &()).unwrap();

                println!("y_k: {}, r_k: {}", y_k.0, r_k);

                //
                // Forecast
                //

                // Augmented state vector = [state_vector_7x1; q1_vector_3x1]
                let mut e = SVector::<f32, 6>::zeros();
                e.fixed_rows_mut::<3>(0).copy_from(x_kk1.euclidean());
                let x_aug = StateVectorAugmented::from_components(x_kk1.quaternion().clone(), e);

                // Augmented state covariance matrix = diag([state_covariance_6x6, multiplacative_process_noise_covariance_3x3])
                let mut p_xx_aug = SMatrix::<f32, 9, 9>::zeros();
                p_xx_aug.fixed_view_mut::<6, 6>(0, 0).copy_from(&p_xx_kk1);
                p_xx_aug.fixed_view_mut::<3, 3>(6, 6).copy_from(&q1);

                // Predict the state and covariance via an unscented transformation of the augmented state + covariance
                // TODO: Fix the .unwrap()
                (x_kk1, p_xx_kk1, _) =
                    Self::ut(Self::process_model, &x_aug, &p_xx_aug, &w, &dt).unwrap();

                // State covariance update
                p_xx_kk1 += q2;

                println!(
                    "x_kk1: {}{}, p_xx_k: {}",
                    x_kk1.quaternion().0,
                    x_kk1.euclidean(),
                    p_xx_kk1
                );

                // Predict measurement
                let (y_kk1, mut p_yy_kk1, p_xy_kk1) =
                    Self::ut(Self::observation_model, &x_kk1, &p_xx_kk1, &(), &()).unwrap();

                // Measurment covariance update
                p_yy_kk1 += r_k;

                println!(
                    "y_kk1: {}, p_yy_kk1: {}, p_xy_kk1: {}",
                    y_kk1.0, p_yy_kk1, p_xy_kk1
                );

                // Innovation
                let v_k = y_k.inner_difference(&y_kk1);

                //
                // Data assimilation
                //

                // Kalman gain
                // TODO: Fix the .unwrap()
                let k_k = p_xy_kk1 * p_yy_kk1.try_inverse().unwrap();

                println!("k_k: {}", k_k);

                // State estimate
                let x_k = x_kk1.inner_displacement(k_k * v_k);

                // Covariance estimate
                let p_xx_k = p_xx_kk1 - k_k * p_yy_kk1 * k_k.transpose();

                println!(
                    "x_k: {}{}, p_xx_k: {}",
                    x_k.quaternion().0,
                    x_k.euclidean(),
                    p_xx_k
                );

                // UPDATE HACK
                x_kk1 = x_k;
                p_xx_kk1 = p_xx_k;

                //
                // Logging
                //
                let (roll, pitch, yaw) = x_kk1.quaternion().0.euler_angles();

                // Raw accl reading (m/s^2)
                rec.log(
                    "accl/raw/x",
                    &rerun::Scalar::new(serial_packet.accl_x as f64),
                )
                .unwrap();
                rec.log(
                    "accl/raw/y",
                    &rerun::Scalar::new(serial_packet.accl_y as f64),
                )
                .unwrap();
                rec.log(
                    "accl/raw/z",
                    &rerun::Scalar::new(serial_packet.accl_z as f64),
                )
                .unwrap();

                // Raw gyro reading (deg/s)
                rec.log(
                    "gyro/raw/roll",
                    &rerun::Scalar::new(serial_packet.gyro_x as f64),
                )
                .unwrap();
                rec.log(
                    "gyro/raw/pitch",
                    &rerun::Scalar::new(serial_packet.gyro_y as f64),
                )
                .unwrap();
                rec.log(
                    "gyro/raw/yaw",
                    &rerun::Scalar::new(serial_packet.gyro_z as f64),
                )
                .unwrap();

                rec.log(
                    "gyro/kalman/roll",
                    &rerun::Scalar::new(roll as f64 * 57.2958),
                )
                .unwrap();
                rec.log(
                    "gyro/kalman/pitch",
                    &rerun::Scalar::new(pitch as f64 * 57.2958),
                )
                .unwrap();
                rec.log("gyro/kalman/yaw", &rerun::Scalar::new(yaw as f64 * 57.2958))
                    .unwrap();

                let x_axis = Vector3::x_axis();
                let y_axis = Vector3::y_axis();
                let z_axis = Vector3::z_axis();

                let x_ = x_kk1.quaternion().0 * x_axis;
                let x_: [f32; 3] = std::array::from_fn(|i| x_[i] as f32);
                let y_ = x_kk1.quaternion().0 * y_axis;
                let y_: [f32; 3] = std::array::from_fn(|i| y_[i] as f32);
                let z_ = x_kk1.quaternion().0 * z_axis;
                let z_: [f32; 3] = std::array::from_fn(|i| z_[i] as f32);

                let accl_ = Vector3::new(
                    serial_packet.accl_x,
                    serial_packet.accl_y,
                    serial_packet.accl_z,
                )
                .normalize();
                let accl_: [f32; 3] = std::array::from_fn(|i| accl_[i] as f32);

                rec.log(
                    "accl/raw/3d",
                    &rerun::Arrows3D::from_vectors([rerun::Vector3D::from(accl_)])
                        .with_origins([rerun::Position3D::ZERO; 1])
                        .with_colors([rerun::Color::from_rgb(255, 255, 0)]),
                )
                .unwrap();

                rec.log(
                    "gyro/kalman/3d",
                    &rerun::Arrows3D::from_vectors([
                        rerun::Vector3D::from(x_),
                        rerun::Vector3D::from(y_),
                        rerun::Vector3D::from(z_),
                    ])
                    .with_origins([rerun::Position3D::ZERO; 3])
                    .with_colors([
                        rerun::Color::from_rgb(255, 0, 0),
                        rerun::Color::from_rgb(0, 255, 0),
                        rerun::Color::from_rgb(0, 0, 255),
                    ]),
                )
                .unwrap();
            }
        })
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("connections")
            .resizable(false)
            .show(ctx, |ui| {
                ui.label("Serial Port:");
                egui::ComboBox::from_id_source("serial_port")
                    .wrap(true)
                    .selected_text(match self.serial_info.as_ref() {
                        Some(p) => p.port_name.clone(),
                        None => "None".to_owned(),
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.serial_info, None, "None");
                        let _ports = match available_ports() {
                            Ok(ports) => {
                                for p in ports {
                                    ui.selectable_value(
                                        &mut self.serial_info,
                                        Some(p.clone()),
                                        p.port_name,
                                    );
                                }
                            }
                            Err(e) => {
                                eprintln!("{:?}", e);
                                eprintln!("Error listing serial ports");
                            }
                        };
                    });

                ui.label("Baud Rate:");
                egui::ComboBox::from_id_source("baud_rate")
                    .selected_text(format!("{}", self.serial_baud.clone() as u32))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.serial_baud, Baud::Br4800, "4800");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br9600, "9600");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br19200, "19200");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br38400, "38400");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br57600, "57600");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br115200, "115200");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br230400, "230400");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br460800, "460800");
                        ui.selectable_value(&mut self.serial_baud, Baud::Br921600, "921600");
                    });

                match self.port {
                    Some(_) => {
                        if ui.button("Disconnect").clicked() {
                            self.port.as_ref().unwrap().abort();
                            self.port = None;
                        }
                    }
                    None => {
                        if ui
                            .add_enabled(
                                match self.serial_info {
                                    Some(_) => true,
                                    None => false,
                                },
                                egui::Button::new("Connect"),
                            )
                            .clicked()
                        {
                            let p = tokio_serial::new(
                                self.serial_info.as_ref().unwrap().port_name.clone(),
                                self.serial_baud.clone() as u32,
                            )
                            .parity(mio_serial::Parity::Even)
                            .stop_bits(mio_serial::StopBits::One)
                            .open_native_async()
                            .unwrap();

                            //let (tx, rx) = channel();
                            self.port = Some(self.test(p, ctx.clone()));
                        }
                    }
                }
            });

        egui::CentralPanel::default().show(ctx, |_| {});
    }
}

struct LineCodec;

impl Decoder for LineCodec {
    type Item = SerialPacket;
    type Error = Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        match src.as_ref().iter().position(|b| *b == 0x00 as u8) {
            Some(zero_pos) => {
                let packet = from_bytes_cobs(src.split_to(zero_pos + 1).as_mut())?;
                return Ok(Some(packet));
            }
            None => Ok(None),
        }
    }
}

#[derive(PartialEq, Clone)]
pub enum Baud {
    Br4800 = 4800,
    Br9600 = 9600,
    Br19200 = 19200,
    Br38400 = 38400,
    Br57600 = 57600,
    Br115200 = 115200,
    Br230400 = 230400,
    Br460800 = 460800,
    Br921600 = 921600,
}
