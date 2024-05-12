#![no_std]

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct SerialPacket {
    pub accl_x: f32,
    pub accl_y: f32,
    pub accl_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}
