use bevy::prelude::*;
use bevy_serial::{SerialPlugin, SerialReadMessage, SerialWriteMessage};

// to write data to serial port periodically
#[derive(Resource)]
struct SerialWriteTimer(Timer);

const SERIAL_PORT: &str = "/dev/ttyUSB0";

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        // simply specify port name and baud rate for `SerialPlugin`
        .add_plugins(SerialPlugin::new(SERIAL_PORT, 115200))
        // to write data to serial port periodically (every 1 second)
        .insert_resource(SerialWriteTimer(Timer::from_seconds(
            1.0,
            TimerMode::Repeating,
        )))
        // reading and writing from/to serial port is achieved via bevy's event system
        .add_systems(Update, read_serial)
        .add_systems(Update, write_serial)
        .run();
}

// reading event for serial port
fn read_serial(mut reader: MessageReader<SerialReadMessage>) {
    // you can get label of the port and received data buffer from `SerialReadMessage`
    for SerialReadMessage(label, buffer) in reader.read() {
        let s = String::from_utf8(buffer.clone()).unwrap();
        println!("Received packet from {label}: {s}");
    }
}

// writing event for serial port
fn write_serial(
    mut writer: MessageWriter<SerialWriteMessage>,
    mut timer: ResMut<SerialWriteTimer>,
    time: Res<Time>,
) {
    // write msg to serial port every 1 second not to flood serial port
    if timer.0.tick(time.delta()).just_finished() {
        // you can write to serial port via `SerialWriteMessage` with label and buffer to write
        let buffer = b"Hello, bevy!";
        writer.write(SerialWriteMessage(SERIAL_PORT.to_string(), buffer.to_vec()));
    }
}
