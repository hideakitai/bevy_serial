use bevy::prelude::*;
use bevy_serial::{SerialPlugin, SerialReadEvent, SerialWriteEvent};

// to write data to serial port periodically
struct SerialWriteTimer(Timer);

fn main() {
    App::build()
        .add_plugins(MinimalPlugins)
        // simply specify port name and baud rate for `SerialPlugin`
        .add_plugin(SerialPlugin::new("COM5", 115200))
        // to write data to serial port periodically (every 1 second)
        .insert_resource(SerialWriteTimer(Timer::from_seconds(1.0, true)))
        // reading and writing from/to serial port is achieved via bevy's event system
        .add_system(read_serial.system())
        .add_system(write_serial.system())
        .run();
}

// reading event for serial port
fn read_serial(mut ev_serial: EventReader<SerialReadEvent>) {
    // you can get label of the port and received data buffer from `SerialReadEvent`
    for SerialReadEvent(label, buffer) in ev_serial.iter() {
        let s = String::from_utf8(buffer.clone()).unwrap();
        println!("received packet from {}: {}", label, s);
    }
}

// writing event for serial port
fn write_serial(
    mut ev_serial: EventWriter<SerialWriteEvent>,
    mut timer: ResMut<SerialWriteTimer>,
    time: Res<Time>,
) {
    if timer.0.tick(time.delta()).just_finished() {
        // you can write to serial port via `SerialWriteEvent` with label and buffer to write
        let buffer = b"Hello, bevy!";
        ev_serial.send(SerialWriteEvent("COM5".to_string(), buffer.to_vec()));
    }
}
