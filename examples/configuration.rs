use bevy::prelude::*;
use bevy_serial::{
    DataBits, FlowControl, Parity, SerialPlugin, SerialReadEvent, SerialSetting, SerialWriteEvent,
    StopBits,
};
use std::time::Duration;

// to write data to serial port periodically
struct SerialWriteTimer(Timer);

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        // you can specify various configurations for multiple serial ports by this way
        .add_plugin(SerialPlugin {
            settings: vec![SerialSetting {
                label: Some("my_serial".to_string()),
                port_name: "COM5".to_string(),
                baud_rate: 115200,
                data_bits: DataBits::Eight,
                flow_control: FlowControl::None,
                parity: Parity::None,
                stop_bits: StopBits::One,
                timeout: Duration::from_millis(0),
            }],
        })
        // to write data to serial port periodically (every 1 second)
        .insert_resource(SerialWriteTimer(Timer::from_seconds(1.0, true)))
        // reading and writing from/to serial port is achieved via bevy's event system
        .add_system(read_serial)
        .add_system(write_serial)
        .run();
}

// reading event for serial port
fn read_serial(mut ev_serial: EventReader<SerialReadEvent>) {
    // you can get label of the port and received data buffer from `SerialReadEvent`
    for SerialReadEvent(label, buffer) in ev_serial.iter() {
        let s = String::from_utf8(buffer.clone()).unwrap();
        println!("read packet from {}: {}", label, s);
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
        ev_serial.send(SerialWriteEvent("my_serial".to_string(), buffer.to_vec()));
    }
}
