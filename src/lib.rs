//! # bevy_serial
//!
//! `bevy_serial` is a plugin to add non-blocking serial communication to bevy. This plugin is based on [`mio-serial`](https://github.com/berkowski/mio-serial) that can realize non-blocking high-performance I/O.
//!
//! Reading and writing from/to serial port is realized via bevy's event system. Each serial port is handled via port name or a unique label you choose. These event handlers are added to the following stage to minimize the frame delay.
//!
//! - Reading: `PreUpdate`
//! - Writing: `PostUpdate`
//!
//! ## Usage
//!
//! ### Simple Example
//!
//! Here is a simple example:
//!
//! ```rust
//! use bevy::prelude::*;
//! use bevy_serial::{SerialPlugin, SerialReadEvent, SerialWriteEvent};
//!
//! // to write data to serial port periodically
//! #[derive(Resource)]
//! struct SerialWriteTimer(Timer);
//!
//! const SERIAL_PORT: &str = "/dev/ttyUSB0";
//!
//! fn main() {
//!     App::new()
//!         .add_plugins(MinimalPlugins)
//!         // simply specify port name and baud rate for `SerialPlugin`
//!         .add_plugins(SerialPlugin::new(SERIAL_PORT, 115200))
//!         // to write data to serial port periodically (every 1 second)
//!         .insert_resource(SerialWriteTimer(Timer::from_seconds(
//!             1.0,
//!             TimerMode::Repeating,
//!         )))
//!         // reading and writing from/to serial port is achieved via bevy's event system
//!         .add_systems(Update, read_serial)
//!         .add_systems(Update, write_serial)
//!         .run();
//! }
//!
//! // reading event for serial port
//! fn read_serial(mut ev_serial: EventReader<SerialReadEvent>) {
//!     // you can get label of the port and received data buffer from `SerialReadEvent`
//!     for SerialReadEvent(label, buffer) in ev_serial.read() {
//!         let s = String::from_utf8(buffer.clone()).unwrap();
//!         println!("received packet from {label}: {s}");
//!     }
//! }
//!
//! // writing event for serial port
//! fn write_serial(
//!     mut ev_serial: EventWriter<SerialWriteEvent>,
//!     mut timer: ResMut<SerialWriteTimer>,
//!     time: Res<Time>,
//! ) {
//!     // write msg to serial port every 1 second not to flood serial port
//!     if timer.0.tick(time.delta()).just_finished() {
//!         // you can write to serial port via `SerialWriteEvent` with label and buffer to write
//!         let buffer = b"Hello, bevy!";
//!         ev_serial.send(SerialWriteEvent(SERIAL_PORT.to_string(), buffer.to_vec()));
//!     }
//! }
//! ```
//!
//! ### Multiple Serial Ports with Additional Config
//!
//! You can add multiple serial ports with additional config.
//!
//! ```rust
//! fn main() {
//!     App::new()
//!         .add_plugins(MinimalPlugins)
//!         // you can specify various configurations for multiple serial ports by this way
//!         .add_plugins(SerialPlugin::new_with_config(vec![SerialConfig {
//!             label: Some(SERIAL_LABEL.to_string()),
//!             port_name: SERIAL_PORT.to_string(),
//!             baud_rate: 115200,
//!             data_bits: DataBits::Eight,
//!             flow_control: FlowControl::None,
//!             parity: Parity::None,
//!             stop_bits: StopBits::One,
//!             timeout: Duration::from_millis(0),
//!             read_buffer_len: 2048,
//!             read_result_handler: Some(Arc::new(|label, result| {
//!                 println!("Read result of {label}: {result:?}");
//!             })),
//!             write_result_handler: Some(Arc::new(|label, result| {
//!                 println!("Write result of {label}: {result:?}");
//!             })),
//!         }]))
//!         // reading and writing from/to serial port is achieved via bevy's event system
//!         .add_systems(Update, read_serial)
//!         .add_systems(Update, write_serial)
//!         .run();
//! }
//! ```
//!
//! ## Supported Versions
//!
//! | bevy  | bevy_serial |
//! | ----- | ----------- |
//! | 0.16  | 0.9         |
//! | 0.15  | 0.8         |
//! | 0.14  | 0.7         |
//! | 0.13  | 0.5, 0.6    |
//! | 0.12  | 0.4         |
//! | 0.11  | 0.3         |
//! | 0.6   | 0.2         |
//! | 0.5   | 0.1         |
//!
//! ## License
//!
//! Dual-licensed under either
//!
//! - MIT
//! - Apache 2.0

pub use mio_serial::{DataBits, FlowControl, Parity, StopBits};

use bevy::app::{App, Plugin, PostUpdate, PreUpdate};
use bevy::ecs::event::{Event, EventReader, EventWriter};
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{In, IntoSystem, Res, ResMut};
use mio::{Events, Interest, Poll, Token};
use mio_serial::SerialStream;
use once_cell::sync::OnceCell;
use std::collections::HashMap;
use std::io::{ErrorKind, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Serial read/write result handler type
type ResultHandler = Arc<dyn Fn(&str, &Result<usize, std::io::Error>) + Send + Sync + 'static>;

/// Config for users to initialize this plugin
#[derive(Clone)]
pub struct SerialConfig {
    /// The intuitive name for this serial port
    pub label: Option<String>,
    /// The port name, usually the device path
    pub port_name: String,
    /// The baud rate in symbols-per-second
    pub baud_rate: u32,
    /// Number of bits used to represent a character sent on the line
    pub data_bits: DataBits,
    /// The type of signalling to use for controlling data transfer
    pub flow_control: FlowControl,
    /// The type of parity to use for error checking
    pub parity: Parity,
    /// Number of bits to use to signal the end of a character
    pub stop_bits: StopBits,
    /// Amount of time to wait to receive data before timing out
    pub timeout: Duration,
    /// The size of read buffer for one read system call
    pub read_buffer_len: usize,
    /// Result handler for read serial port
    pub read_result_handler: Option<ResultHandler>,
    /// Result handler for write serial port
    pub write_result_handler: Option<ResultHandler>,
}

impl std::fmt::Debug for SerialConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SerialConfig")
            .field("label", &self.label)
            .field("port_name", &self.port_name)
            .field("baud_rate", &self.baud_rate)
            .field("data_bits", &self.data_bits)
            .field("flow_control", &self.flow_control)
            .field("parity", &self.parity)
            .field("stop_bits", &self.stop_bits)
            .field("timeout", &self.timeout)
            .field("read_buffer_len", &self.read_buffer_len)
            .finish()
    }
}

impl Default for SerialConfig {
    fn default() -> Self {
        Self {
            label: None,
            port_name: "".to_string(),
            baud_rate: 115200,
            data_bits: DataBits::Eight,
            flow_control: FlowControl::None,
            parity: Parity::None,
            stop_bits: StopBits::One,
            timeout: Duration::from_millis(0),
            read_buffer_len: 2048,
            read_result_handler: None,
            write_result_handler: None,
        }
    }
}

/// Bevy's event type to read serial port
#[derive(Event)]
pub struct SerialReadEvent(pub String, pub Vec<u8>);

/// Bevy's event type to read serial port
#[derive(Event)]
pub struct SerialWriteEvent(pub String, pub Vec<u8>);

/// Serial struct that is used internally for this crate
#[derive(Debug)]
struct SerialStreamLabeled {
    stream: SerialStream,
    label: String,
    read_buffer_len: usize,
}

/// Context to poll serial read event with `Poll` in `mio` crate
#[derive(Resource)]
struct MioContext {
    poll: Poll,
    events: Events,
}

impl MioContext {
    /// poll serial read event (should timeout not to block other systems)
    fn poll(&mut self) {
        self.poll
            .poll(&mut self.events, Some(Duration::from_micros(1)))
            .unwrap_or_else(|e| {
                panic!("Failed to poll events: {e:?}");
            });
    }
}

/// Component to get an index of serial port based on the label
#[derive(Resource)]
struct Indices(HashMap<String, usize>);

/// Serial read/write results passed to the result handler
type ReadWriteResults = HashMap<String, Result<usize, std::io::Error>>;

/// Module scope global singleton to store serial ports
static SERIALS: OnceCell<Vec<Mutex<SerialStreamLabeled>>> = OnceCell::new();
/// Module scope global singleton to store read serial result handler
static READ_RESULT_HANDLERS: OnceCell<HashMap<String, Option<ResultHandler>>> = OnceCell::new();
/// Module scope global singleton to store write serial result handler
static WRITE_RESULT_HANDLERS: OnceCell<HashMap<String, Option<ResultHandler>>> = OnceCell::new();

/// Plugin that can be added to Bevy
#[derive(Clone)]
pub struct SerialPlugin {
    pub config: Vec<SerialConfig>,
}

impl SerialPlugin {
    pub fn new(port_name: &str, baud_rate: u32) -> Self {
        Self {
            config: vec![SerialConfig {
                port_name: port_name.to_string(),
                baud_rate,
                ..Default::default()
            }],
        }
    }

    pub fn new_with_config(config: Vec<SerialConfig>) -> Self {
        Self { config }
    }
}

impl Plugin for SerialPlugin {
    fn build(&self, app: &mut App) {
        let poll = Poll::new().unwrap();
        let events = Events::with_capacity(self.config.len());
        let mio_ctx = MioContext { poll, events };
        let mut serials: Vec<Mutex<SerialStreamLabeled>> = vec![];
        let mut read_result_handlers: HashMap<String, Option<ResultHandler>> = HashMap::new();
        let mut write_result_handlers: HashMap<String, Option<ResultHandler>> = HashMap::new();
        let mut indices = Indices(HashMap::new());

        for (i, config) in self.config.iter().enumerate() {
            // create serial port builder from `serialport` crate
            let port_builder = serialport::new(&config.port_name, config.baud_rate)
                .data_bits(config.data_bits)
                .flow_control(config.flow_control)
                .parity(config.parity)
                .stop_bits(config.stop_bits)
                .timeout(config.timeout);

            // create `mio_serial::SerailStream` from `seriaport` builder
            let mut stream = SerialStream::open(&port_builder).unwrap_or_else(|e| {
                panic!("Failed to open serial port {}: {:?}", config.port_name, e);
            });

            // token index is same as index of vec
            mio_ctx
                .poll
                .registry()
                .register(&mut stream, Token(i), Interest::READABLE)
                .unwrap_or_else(|e| {
                    panic!("Failed to register stream to poll : {e:?}");
                });

            // if label is set, use label as a nickname of serial
            // if not, use `port_name` as a nickname
            let label = if let Some(label) = &config.label {
                label.clone()
            } else {
                config.port_name.clone()
            };

            // store indices (resource)
            indices.0.insert(label.clone(), i);
            // store serials and result handlers (global variables)
            read_result_handlers.insert(label.clone(), config.read_result_handler.clone());
            write_result_handlers.insert(label.clone(), config.write_result_handler.clone());
            serials.push(Mutex::new(SerialStreamLabeled {
                stream,
                label,
                read_buffer_len: config.read_buffer_len,
            }));
        }

        // set to global variables lazily
        SERIALS.set(serials).unwrap_or_else(|e| {
            panic!("Failed to set SerialStream to global variable: {e:?}");
        });
        READ_RESULT_HANDLERS
            .set(read_result_handlers)
            .unwrap_or_else(|_| {
                panic!("Failed to set read result handler to global variable");
            });
        WRITE_RESULT_HANDLERS
            .set(write_result_handlers)
            .unwrap_or_else(|_| {
                panic!("Failed to set write result handler to global variable");
            });

        app.insert_resource(mio_ctx)
            .insert_resource(indices)
            .add_event::<SerialReadEvent>()
            .add_event::<SerialWriteEvent>()
            .add_systems(PreUpdate, read_serial.pipe(read_serial_result_handler))
            .add_systems(PostUpdate, write_serial.pipe(write_serial_result_handler));
    }
}

/// Poll serial read event with `Poll` in `mio` crate.
/// If any data has come to serial, `SerialReadEvent` is sent to the system subscribing it.
fn read_serial(
    mut ev_receive_serial: EventWriter<SerialReadEvent>,
    mut mio_ctx: ResMut<MioContext>,
    indices: Res<Indices>,
) -> ReadWriteResults {
    if indices.0.is_empty() {
        return ReadWriteResults::new();
    }

    // poll serial read events
    mio_ctx.poll();

    // if events have occurred, send `SerialReadEvent` with serial labels and read data buffer
    let mut read_results = ReadWriteResults::new();
    for event in mio_ctx.events.iter() {
        if !event.is_readable() {
            continue;
        }

        // get serial instance based on the token index
        let serials = SERIALS.get().expect("SERIALS are not initialized");
        let serial_mtx = serials
            .get(event.token().0) // token index is same as index of vec
            .expect(&format!("SERIALS index {} not found", event.token().0));

        // try to get lock of mutex and send data to event
        if let Ok(mut serial) = serial_mtx.lock() {
            let mut buffer = vec![0_u8; serial.read_buffer_len];
            let mut bytes_read = 0;
            loop {
                match serial.stream.read(&mut buffer[bytes_read..]) {
                    Ok(0) => {
                        eprintln!("{} connection maybe closed", serial.label);
                        read_results.insert(
                            serial.label.clone(),
                            Err(std::io::Error::new(
                                ErrorKind::NotConnected,
                                "Maybe connection closed",
                            )),
                        );
                        break;
                    }
                    // read data successfully
                    // if buffer is full, maybe there is more data to read
                    Ok(n) => {
                        bytes_read += n;
                        if bytes_read == buffer.len() {
                            buffer.resize(buffer.len() + serial.read_buffer_len, 0);
                        }
                        continue;
                    }
                    // would block indicates no more data to read
                    Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                        let label = serial.label.clone();
                        let buffer = buffer.drain(..bytes_read).collect();
                        ev_receive_serial.write(SerialReadEvent(label, buffer));
                        read_results.insert(serial.label.clone(), Ok(bytes_read));
                        break;
                    }
                    // if interrupted, we should continue readings
                    Err(ref e) if e.kind() == ErrorKind::Interrupted => {
                        continue;
                    }
                    // other errors are fatal
                    Err(e) => {
                        eprintln!("Failed to read serial port {}: {}", serial.label, e);
                        read_results.insert(serial.label.clone(), Err(e));
                        break;
                    }
                }
            }
        }
    }
    read_results
}

/// Read serial result handler to handle read results
/// This system is piped with `read_serial` system
fn read_serial_result_handler(In(result): In<ReadWriteResults>) {
    let read_result_handlers = READ_RESULT_HANDLERS
        .get()
        .expect("READ_RESULT_HANDLERS are not initialized");

    for (label, result) in result.iter() {
        if let Some(Some(handler)) = read_result_handlers.get(label) {
            handler(label, result);
        }
    }
}

/// Write bytes to serial port.
/// The bytes are sent via `SerialWriteEvent` with label of serial port.
fn write_serial(
    mut ev_write_serial: EventReader<SerialWriteEvent>,
    indices: Res<Indices>,
) -> ReadWriteResults {
    if indices.0.is_empty() {
        return ReadWriteResults::new();
    }

    let mut rw_results = ReadWriteResults::new();
    for SerialWriteEvent(label, buffer) in ev_write_serial.read() {
        // get index of label
        let &serial_index = indices.0.get(label).unwrap_or_else(|| {
            panic!("{} is not exist", label.as_str());
        });
        let serials = SERIALS.get().expect("SERIALS are not initialized");
        let serial_mtx = serials
            .get(serial_index)
            .expect(&format!("SERIALS index {serial_index} not found"));

        // try to get lock of mutex and send data to event
        if let Ok(mut serial) = serial_mtx.lock() {
            // write buffered data to serial
            let mut bytes_wrote = 0;
            loop {
                // write the entire buffered data in a single system call
                match serial.stream.write(&buffer[bytes_wrote..]) {
                    // error if returned len is less than expected (same as `io::Write::write_all` does)
                    Ok(n) if n < buffer.len() => {
                        bytes_wrote += n;
                    }
                    // wrote queued data successfully
                    Ok(_) => {
                        bytes_wrote += buffer.len();
                    }
                    // would block indicates that this port is not ready so try again
                    Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}
                    // if interrupted, we should try again
                    Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                    // other errors are fatal
                    Err(e) => {
                        eprintln!("Failed to write serial port {}: {}", serial.label, e);
                        rw_results.insert(serial.label.clone(), Err(e));
                        break;
                    }
                }

                if bytes_wrote == buffer.len() {
                    rw_results.insert(serial.label.clone(), Ok(bytes_wrote));
                    break;
                } else {
                    continue;
                }
            }
        }
    }
    rw_results
}

/// Write serial result handler to handle write results
/// This system is piped with `write_serial` system
fn write_serial_result_handler(In(result): In<ReadWriteResults>) {
    let write_result_handlers = WRITE_RESULT_HANDLERS
        .get()
        .expect("WRITE_RESULT_HANDLERS are not initialized");

    for (label, result) in result.iter() {
        if let Some(Some(handler)) = write_result_handlers.get(label) {
            handler(label, result);
        }
    }
}
