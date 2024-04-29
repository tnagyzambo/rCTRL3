use bytes::{Buf, BytesMut};
use eframe::egui;
use futures::stream::StreamExt;
use rerun::RecordingStream;
use std::{io::Read, sync::mpsc::channel};
use tokio::task::JoinHandle;
use tokio_serial::{available_ports, SerialPortBuilderExt, SerialPortInfo, SerialStream};
use tokio_util::codec::{Decoder, Encoder};

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
    rec: RecordingStream,
    serial_info: Option<SerialPortInfo>,
    serial_baud: Baud,
    port: Option<JoinHandle<()>>,
}

impl App {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        // Rerun recorder
        let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal")
            .spawn()
            .unwrap();

        Self {
            rec,
            serial_info: None,
            serial_baud: Baud::Br57600,
            port: None,
        }
    }

    fn test(&self, p: SerialStream, ctx: egui::Context) -> JoinHandle<()> {
        tokio::spawn(async move {
            let mut reader = LineCodec.framed(p);
            while let Some(line_result) = reader.next().await {
                let line = line_result.expect("Failed to read line");
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
    type Item = String;
    type Error = std::io::Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let newline = src.as_ref().iter().position(|b| *b == b'\n');
        if let Some(n) = newline {
            let line = src.split_to(n + 1);
            let gyro_x = f32::from_be_bytes([line[0], line[1], line[2], line[3]]);
            let gyro_y = f32::from_be_bytes([line[4], line[5], line[6], line[7]]);
            let gyro_z = f32::from_be_bytes([line[8], line[9], line[10], line[11]]);
            println!("X: {}, Y: {}, Z: {}", gyro_x, gyro_y, gyro_z);
        }
        Ok(None)
    }
}
