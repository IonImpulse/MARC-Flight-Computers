use chrono::*;
use serialport;
use serialport::*;
use std::io::{self, Write, Read, BufRead, BufReader};
use std::time::Duration;

sixtyfps::include_modules!();

enum RocketState {
    Ready,
    Armed,
    InFlight,
    Parachuted,
    Landed,
}

/// Represents the transmitter location history
/// String format:
///    [state, timestamp, gps_latitude, gps_longitude, gps_altitude, gps_speed, altimeter_altitude]
///
struct Location {
    pub raw_sentence: String,
    pub state: RocketState,
    pub timestamp: DateTime<Utc>,
    pub gps_latitude: f64,
    pub gps_longitude: f64,
    pub gps_altitude: Option<f64>,
    pub gps_speed: Option<f64>,
    pub altimeter_altitude: Option<f64>,
}

struct Receiver {
    serial_connection: Box<serialport::SerialPort>,
    location: Location,
}

struct Transmitter {
    callsign: String,
    location_history: Vec<Location>,
    has_gps: bool,
    has_altimeter: bool,
    has_accelerometer: bool,
}

fn find_receiver() -> Option<serialport::SerialPortInfo> {
    // First, get a list of all serial ports available on the system
    let avail_ports = serialport::available_ports().unwrap();

    // Iterate over the available ports and find the first one that matches the
    // serial port name we're looking for
    for port in avail_ports {
        print!(
            "PORT {} \n==========\n{:?}\n==========\n",
            port.port_name, port.port_type
        );
        let port_info = format!("{:?}", port.port_type).as_str().to_lowercase();
        if port_info.contains("usb serial device") {
            println!("FOUND {}", &port.port_name);
            return Some(port);
        }
    }

    return None;
}

fn hex_str(bytes: &[u8]) -> String {
    let mut hex = String::from("");
    let mut ascii = String::from("");

    for byte in bytes.iter() {
        hex.push_str(&format!("{:02x} ", *byte));

        if *byte < 0x20 {
            if *byte == 0x1b {
                if !ascii.is_empty() {
                    ascii.push(' ');
                }
                ascii.push_str("ESC");
            } else {
                if !ascii.is_empty() {
                    ascii.push(' ');
                }
                ascii.push_str("Ctrl-");
                let ctrl = b"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_";
                ascii.push(ctrl[*byte as usize] as char);
            }
        } else if *byte > b'~' {
            ascii.push('.');
        } else {
            ascii.push(*byte as char);
        }
    }

    hex.push(':');
    hex.push(' ');
    hex.push_str(&ascii);
    hex
}

fn main() {
    // First, find the serial port
    let port_info = find_receiver();

    // If we didn't find a port, then we can't continue
    if port_info.is_none() {
        panic!("Could not find a serial port");
    }

    let mut port = serialport::new(port_info.unwrap().port_name, 115200)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(Duration::from_millis(1000))
        .open()
        .expect("Could not open serial port");

    // NEEDED to make the serial port work on windows
    // stupid that this is necessary
    port.write_data_terminal_ready(true);

    let mut reader = BufReader::new(port);

    loop {
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();

        println!("Buffer is {:?}", line);
    }

    let ui = AppWindow::new();

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });

    ui.run();
}
