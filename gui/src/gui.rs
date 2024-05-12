use anyhow::Error;
use bytes::BytesMut;
use eframe::egui;
use futures::stream::StreamExt;
use na::{Matrix4, Quaternion, Rotation3, UnitQuaternion, Vector3, Vector4};
use nalgebra as na;
use postcard::from_bytes_cobs;
use rctrl_lib::SerialPacket;
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

            // Kalman
            let mut x = Vector4::<f32>::new(1.0, 0.0, 0.0, 0.0);
            let mut p = Matrix4::<f32>::identity();
            let h = Matrix4::<f32>::identity();
            let q = 0.0001 * Matrix4::<f32>::identity();
            let r = 10.0 * Matrix4::<f32>::identity();

            loop {
                let serial_packet = match reader.next().await {
                    Some(p) => p,
                    None => continue,
                };

                let serial_packet = match serial_packet {
                    Ok(p) => p,
                    Err(e) => {
                        println!("{}", e);
                        continue;
                    }
                };

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

                let dt = t.elapsed().as_millis() as f32 / 1000.0;
                t = Instant::now();

                // Kalman filter
                let w1 = serial_packet.gyro_x * 0.0174533;
                let w2 = serial_packet.gyro_y * 0.0174533;
                let w3 = serial_packet.gyro_z * 0.0174533;

                let b = Matrix4::<f32>::new(
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

                let x_ = rot * x_axis;
                let x_: [f32; 3] = std::array::from_fn(|i| x_[i] as f32);
                let y_ = rot * y_axis;
                let y_: [f32; 3] = std::array::from_fn(|i| y_[i] as f32);
                let z_ = rot * z_axis;
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
