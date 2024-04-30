use bytes::BytesMut;
use eframe::egui;
use futures::stream::StreamExt;
use na::{Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3, Vector4};
use nalgebra as na;
use std::time::Instant;
use tokio::task::JoinHandle;
use tokio_serial::{available_ports, SerialPortBuilderExt, SerialPortInfo, SerialStream};
use tokio_util::codec::Decoder;

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

    fn test(&self, serial: SerialStream, ctx: egui::Context) -> JoinHandle<()> {
        tokio::spawn(async move {
            let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal")
                .spawn()
                .unwrap();
            let mut reader = LineCodec.framed(serial);
            let mut t = Instant::now();
            let mut roll = 0.0;
            let mut pitch = 0.0;
            let mut yaw = 0.0;
            let mut skip = 0;

            // Kalman
            let mut x = Vector4::<f64>::new(1.0, 0.0, 0.0, 0.0);
            let mut p = Matrix4::<f64>::identity();
            let h = Matrix4::<f64>::identity();
            let q = 0.0001 * Matrix4::<f64>::identity();
            let r = 10.0 * Matrix4::<f64>::identity();

            while let Some(line_result) = reader.next().await {
                if skip != 2 {
                    skip += 1;
                } else {
                    let gyro = line_result.expect("Failed to read line");

                    // Raw gyro reading (deg/s)
                    rec.log("raw/gyro_x", &rerun::Scalar::new(gyro[0] as f64))
                        .unwrap();
                    rec.log("raw/gyro_y", &rerun::Scalar::new(gyro[1] as f64))
                        .unwrap();
                    rec.log("raw/gyro_z", &rerun::Scalar::new(gyro[2] as f64))
                        .unwrap();

                    let dt = t.elapsed().as_millis() as f64 / 1000.0;
                    t = Instant::now();

                    // Naive attitude (deg)
                    roll += gyro[0] as f64 * dt;
                    pitch += gyro[1] as f64 * dt;
                    yaw += gyro[2] as f64 * dt;

                    rec.log("int/roll", &rerun::Scalar::new(roll)).unwrap();
                    rec.log("int/pitch", &rerun::Scalar::new(pitch)).unwrap();
                    rec.log("int/yaw", &rerun::Scalar::new(yaw)).unwrap();

                    let x_axis = Vector3::x_axis();
                    let y_axis = Vector3::y_axis();
                    let z_axis = Vector3::z_axis();
                    let rot = Rotation3::from_euler_angles(
                        roll * 0.0174533,
                        pitch * 0.0174533,
                        yaw * 0.0174533,
                    );
                    let x_ = rot * x_axis;
                    let x_: [f32; 3] = std::array::from_fn(|i| x_[i] as f32);
                    let y_ = rot * y_axis;
                    let y_: [f32; 3] = std::array::from_fn(|i| y_[i] as f32);
                    let z_ = rot * z_axis;
                    let z_: [f32; 3] = std::array::from_fn(|i| z_[i] as f32);

                    rec.log(
                        "int/3d",
                        &rerun::Arrows3D::from_vectors([
                            rerun::Vector3D::from(x_),
                            rerun::Vector3D::from(y_),
                            rerun::Vector3D::from(z_),
                        ])
                        .with_origins([rerun::Position3D::ZERO; 3])
                        .with_colors([
                            rerun::Color::from_unmultiplied_rgba(255, 0, 0, 50),
                            rerun::Color::from_unmultiplied_rgba(0, 255, 0, 50),
                            rerun::Color::from_unmultiplied_rgba(0, 0, 255, 50),
                        ]),
                    )
                    .unwrap();

                    // Kalman filter
                    let w1 = gyro[0] as f64 * 0.0174533;
                    let w2 = gyro[1] as f64 * 0.0174533;
                    let w3 = gyro[2] as f64 * 0.0174533;

                    let b = Matrix4::<f64>::new(
                        0.0, -w1, -w2, -w3, //
                        w1, 0.0, w3, -w2, //
                        w2, -w3, 0.0, w1, //
                        w3, w2, -w1, 0.0,
                    );

                    let a = Matrix4::identity() + dt * 0.5 * b;

                    let xp = a * x;
                    let pp = a * p * a.transpose() + q;

                    let k = pp
                        * h.transpose()
                        * (h * pp * h.transpose() + r)
                            .try_inverse()
                            .expect("inverse failed");

                    x = xp + k * (x - h * xp);
                    p = pp - k * h * pp;

                    let q = Quaternion::new(x[0], x[1], x[2], x[3]);
                    let rot = UnitQuaternion::from_quaternion(q);

                    let (roll, pitch, yaw) = rot.euler_angles();

                    rec.log("kalman/roll", &rerun::Scalar::new(roll * 57.2958))
                        .unwrap();
                    rec.log("kalman/pitch", &rerun::Scalar::new(pitch * 57.2958))
                        .unwrap();
                    rec.log("kalman/yaw", &rerun::Scalar::new(yaw * 57.2958))
                        .unwrap();

                    let x_ = rot * x_axis;
                    let x_: [f32; 3] = std::array::from_fn(|i| x_[i] as f32);
                    let y_ = rot * y_axis;
                    let y_: [f32; 3] = std::array::from_fn(|i| y_[i] as f32);
                    let z_ = rot * z_axis;
                    let z_: [f32; 3] = std::array::from_fn(|i| z_[i] as f32);

                    rec.log(
                        "kalman/3d",
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
            }
            ctx.request_repaint();
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
                        let ports = match available_ports() {
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

        egui::CentralPanel::default().show(ctx, |ui| {});
    }
}

struct LineCodec;

impl Decoder for LineCodec {
    type Item = [f32; 3];
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let mut prev_byte_newline = false;
        let newline = src.as_ref().iter().position(|b| {
            if *b == 0x0D as u8 && prev_byte_newline {
                return true;
            }

            if *b == 0x0A as u8 {
                prev_byte_newline = true;
            } else {
                prev_byte_newline = false;
            }

            false
        });
        if let Some(n) = newline {
            let p = src.split_to(n + 1);
            let gyro_x = f32::from_be_bytes([p[0], p[1], p[2], p[3]]);
            let gyro_y = f32::from_be_bytes([p[4], p[5], p[6], p[7]]);
            let gyro_z = f32::from_be_bytes([p[8], p[9], p[10], p[11]]);
            return Ok(Some([gyro_x, gyro_y, gyro_z]));
        }
        Ok(None)
    }
}
