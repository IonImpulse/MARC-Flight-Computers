//#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use serialport;
use serialport::*;

use lazy_static::lazy_static;

use std::io::{self, BufRead, BufReader, Read, Write};
use std::thread;
use std::time::Duration;
use std::{collections::*, env, sync::Arc, sync::Mutex, sync::MutexGuard, fs};

use chrono::*;

use plotters::prelude::*;
use sixtyfps::{Image, SharedPixelBuffer, SharedString};

use serde::{Deserialize, Serialize};
use serde_derive::*;

sixtyfps::include_modules!();

const LOG_FILE_NAME: &str = "ground-station.log";

struct GlobalState {
    pub has_receiver: bool,
    pub transmitters: HashMap<u16, Transmitter>,
    pub location: Option<Location>,
    pub active_rocket_id: u16,
}

impl GlobalState {
    pub fn new() -> GlobalState {
        GlobalState {
            has_receiver: false,
            transmitters: HashMap::new(),
            location: None,
            active_rocket_id: 0,
        }
    }
}

#[derive(Debug, Clone)]
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
#[derive(Debug, Clone)]
struct Location {
    pub raw_sentence: String,
    pub id: u16,
    pub state: RocketState,
    pub signal_strength: i16,
    pub timestamp: DateTime<Utc>,
    pub gps_solve_status: Option<u8>,
    pub gps_latitude: Option<f64>,
    pub gps_longitude: Option<f64>,
    pub gps_altitude: Option<f64>,
    pub gps_speed: Option<f64>,
    pub gps_satellites: Option<u8>,
    pub gps_heading: Option<f64>,
    pub x_accel: Option<f64>,
    pub y_accel: Option<f64>,
    pub z_accel: Option<f64>,
    pub altimeter_altitude: Option<f64>,
}

fn to_decimal_degrees(ddmmmmmm: f64) -> f64 {
    let degrees = (ddmmmmmm / 100.0).floor();
    let minutes = ddmmmmmm - (degrees * 100.0);

    degrees + (minutes / 60.0)
}

fn combine_timestamps(
    timestamp_ddmmyy: f64,
    timestamp_hhmmss_sss: f64,
) -> std::result::Result<DateTime<Utc>, Box<dyn std::error::Error>> {
    let day = (timestamp_ddmmyy / 10000.0).floor() as f64;
    let month = ((timestamp_ddmmyy - (day * 10000.0)) / 100.0).floor() as f64;
    // TODO: fix when year is > 2099
    let year = 2000.0 + (timestamp_ddmmyy - (day * 10000.0) - (month * 100.0)) as f64;

    let hours = (timestamp_hhmmss_sss / 10000.0).floor() as f64;
    let minutes = ((timestamp_hhmmss_sss - (hours * 10000.0)) / 100.0).floor() as f64;
    let seconds = (timestamp_hhmmss_sss - (hours * 10000.0) - (minutes * 100.0)).floor() as f64;

    if year < 2000.0
        || year > 2099.0
        || month < 1.0
        || month > 12.0
        || day < 1.0
        || day > 31.0
        || hours < 0.0
        || hours > 23.0
        || minutes < 0.0
        || minutes > 59.0
        || seconds < 0.0
        || seconds > 59.0
    {
        return Err("Invalid timestamp".into());
    }

    let combined_timestamp = DateTime::<Utc>::from_utc(
        NaiveDate::from_ymd(year as i32, month as u32, day as u32).and_hms(
            hours as u32,
            minutes as u32,
            seconds as u32,
        ),
        Utc,
    );

    Ok(combined_timestamp)
}

impl Location {
    pub fn new_empty() -> Self {
        Location {
            raw_sentence: String::new(),
            id: 0,
            state: RocketState::Ready,
            signal_strength: 0,
            timestamp: Utc::now(),
            gps_solve_status: None,
            gps_latitude: None,
            gps_longitude: None,
            gps_altitude: None,
            gps_speed: None,
            gps_satellites: None,
            gps_heading: None,
            x_accel: None,
            y_accel: None,
            z_accel: None,
            altimeter_altitude: None,
        }
    }

    pub fn new_from_line(line: String) -> std::result::Result<Self, Box<dyn std::error::Error>> {
        let mut location = Location::new_empty();

        // First, find the last "|" in the string
        let mut last_pipe_index = 0;
        for (i, c) in line.chars().enumerate() {
            if c == '|' {
                last_pipe_index = i;
            }
        }

        // Then, split the string at the last "|"
        let (_, sentence) = line.split_at(last_pipe_index + 1);

        // Get signal strength
        location.signal_strength = sentence.split(',').nth(1).ok_or("error")?.parse::<i16>()?;

        // Split by $groups
        let mut groups = sentence.split('$');

        for group in groups {
            let mut fields = group.split(',');

            let nmea_type = fields.nth(0).ok_or("error")?;

            if nmea_type == "GNRMC" {
                let timestamp_hhmmss_sss = fields.next().ok_or("error")?.parse::<f64>()?;
                let fix_type = fields.next().ok_or("error")?;

                if fix_type == "V" {
                    return Err("No GPS fix".into());
                }
                // To get lat/lon in correct format, we need to change from
                // ddmm.mmmm & N/S to +/-dd.dddddd

                // First get into dd.mmmmmm format
                let gps_latitude =
                    to_decimal_degrees(fields.next().ok_or("error")?.parse::<f64>()?);

                let gps_latitude_direction = fields.next().ok_or("error")?;
                location.gps_latitude = match gps_latitude_direction.as_ref() {
                    "N" => Some(gps_latitude),
                    "S" => Some(-gps_latitude),
                    _ => return Err("error".into()),
                };

                let gps_longitude =
                    to_decimal_degrees(fields.next().ok_or("error")?.parse::<f64>()?);

                let gps_longitude_direction = fields.next().ok_or("error")?;
                location.gps_longitude = match gps_longitude_direction.as_ref() {
                    "E" => Some(gps_longitude),
                    "W" => Some(-gps_longitude),
                    _ => return Err("error".into()),
                };

                location.gps_speed = fields.next().ok_or("error")?.parse::<f64>().ok();
                location.gps_heading = fields.next().ok_or("error")?.parse::<f64>().ok();

                let timestamp_ddmmyy = fields.next().ok_or("error")?.parse::<f64>().unwrap_or(0.0);

                location.timestamp = combine_timestamps(timestamp_ddmmyy, timestamp_hhmmss_sss)?;
            } else if nmea_type == "GNGGA" {
                let timestamp_hhmmss_sss = fields.next().ok_or("error")?.parse::<f64>()?;

                let gps_latitude =
                    to_decimal_degrees(fields.next().ok_or("error")?.parse::<f64>()?);

                let gps_latitude_direction = fields.next().ok_or("error")?;
                location.gps_latitude = match gps_latitude_direction.as_ref() {
                    "N" => Some(gps_latitude),
                    "S" => Some(-gps_latitude),
                    _ => return Err("error".into()),
                };

                let gps_longitude =
                    to_decimal_degrees(fields.next().unwrap().parse::<f64>().unwrap_or(0.0));

                let gps_longitude_direction = fields.next().ok_or("error")?;
                location.gps_longitude = match gps_longitude_direction.as_ref() {
                    "E" => Some(gps_longitude),
                    "W" => Some(-gps_longitude),
                    _ => return Err("error".into()),
                };

                location.gps_solve_status = fields.next().ok_or("error")?.parse::<u8>().ok();
                location.gps_satellites = fields.next().ok_or("error")?.parse::<u8>().ok();
                let _gps_precision = fields.next().ok_or("error")?.parse::<f64>().ok();

                location.gps_altitude = fields.next().ok_or("error")?.parse::<f64>().ok();

                let utc: DateTime<Utc> = Utc::now();
                let timestamp_ddmmyy =
                    format!("{:02}{:02}{}", utc.day(), utc.month(), utc.year() - 2000)
                        .parse::<f64>()?;

                location.timestamp = combine_timestamps(timestamp_ddmmyy, timestamp_hhmmss_sss)?;
            } else if nmea_type == "ACCEL" {
                location.x_accel = fields.next().ok_or("error")?.parse::<f64>().ok();
                location.y_accel = fields.next().ok_or("error")?.parse::<f64>().ok();
                location.z_accel = fields.next().ok_or("error")?.parse::<f64>().ok();
            } else if nmea_type == "ALT" {
                location.altimeter_altitude = fields.next().ok_or("error")?.parse::<f64>().ok();
            }
        }

        Ok(location)
    }

    pub fn as_raw(&self) -> &str {
        &self.raw_sentence
    }

    pub fn to_sixtyfps_loc(&self) -> activeLocation {
        activeLocation {
            timestamp: SharedString::from(self.timestamp.format("%Y-%m-%dT%H:%M:%SZ").to_string()),
            gps_latitude: self.gps_latitude.unwrap_or(0.0) as f32,
            gps_longitude: self.gps_longitude.unwrap_or(0.0) as f32,
            gps_altitude: self.gps_altitude.unwrap() as f32,
            gps_speed: self.gps_speed.unwrap() as f32,
            gps_satellites: self.gps_satellites.unwrap() as i32,
            gps_solve_status: match self.gps_solve_status.unwrap() {
                0 => SharedString::from("No Fix"),
                1 => SharedString::from("2D Solve"),
                2 => SharedString::from("3D Solve"),
                _ => SharedString::from("Error"),
            },
            altimeter_altitude: self.altimeter_altitude.unwrap_or(0.0) as f32,
            x_accel: self.x_accel.unwrap_or(0.0) as f32,
            y_accel: self.y_accel.unwrap_or(0.0) as f32,
            z_accel: self.z_accel.unwrap_or(0.0) as f32,
        }
    }
}

impl ToString for Location {
    // String representation of all fields in the Location struct
    fn to_string(&self) -> String {
        let mut to_return = String::new();

        to_return.push_str("Location: {");
        to_return.push_str(&format!("\n\tid: {}", self.id));
        to_return.push_str(&format!("\n\tsignal_strength: {}", self.signal_strength));
        to_return.push_str(&format!("\n\tstate: {:?}", self.state));
        to_return.push_str(&format!("\n\ttimestamp: {}", self.timestamp));
        to_return.push_str(&format!(
            "\n\tgps_solve_status: {:?}",
            self.gps_solve_status
        ));
        to_return.push_str(&format!("\n\tgps_latitude: {:?}", self.gps_latitude));
        to_return.push_str(&format!("\n\tgps_longitude: {:?}", self.gps_longitude));
        to_return.push_str(&format!("\n\tgps_altitude: {:?}", self.gps_altitude));
        to_return.push_str(&format!("\n\tgps_speed: {:?}", self.gps_speed));
        to_return.push_str(&format!("\n\tgps_satellites: {:?}", self.gps_satellites));
        to_return.push_str(&format!(
            "\n\taltimeter_altitude: {:?}",
            self.altimeter_altitude
        ));
        to_return.push_str(&format!("\n\tx_accel: {:?}", self.x_accel));
        to_return.push_str(&format!("\n\ty_accel: {:?}", self.y_accel));
        to_return.push_str(&format!("\n\tz_accel: {:?}", self.z_accel));
        to_return.push_str("\n}");

        to_return
    }
}

#[derive(Debug, Clone)]
struct Transmitter {
    id: u16,
    location_history: Vec<Location>,
    ping_history: Vec<chrono::DateTime<Utc>>,
    has_gps: bool,
    has_altimeter: bool,
    has_accelerometer: bool,
}

impl Transmitter {
    pub fn get_last_full_loc(&self) -> Option<Location> {
        let mut valid_gnrmc: Option<Location> = None;
        let mut valid_gngga: Option<Location> = None;
        let mut valid_alt: Option<Location> = None;
        let mut valid_accel: Option<Location> = None;

        let mut reversed = self.location_history.clone();
        reversed.reverse();

        for location in reversed {
            if location.gps_speed.is_some() && valid_gnrmc.is_none() {
                valid_gnrmc = Some(location.clone());
            }
            
            if location.gps_altitude.is_some() && valid_gngga.is_none() {
                valid_gngga = Some(location.clone());
            } 

            if location.altimeter_altitude.is_some() && valid_alt.is_none() {
                valid_alt = Some(location.clone());
            }

            if location.x_accel.is_some() && valid_accel.is_none() {
                valid_accel = Some(location.clone());
            }

            if valid_gnrmc.is_some() && valid_gngga.is_some() && valid_alt.is_some() && valid_accel.is_some() {
                // Combine the two
                let valid_gngga = valid_gngga.unwrap();
                let valid_alt = valid_alt.unwrap();
                let valid_accel = valid_accel.unwrap();

                let mut combined_location = valid_gnrmc.unwrap();
                combined_location.gps_altitude = valid_gngga.gps_altitude;
                combined_location.gps_satellites = valid_gngga.gps_solve_status;
                combined_location.gps_solve_status = valid_gngga.gps_solve_status;
                combined_location.altimeter_altitude = valid_alt.altimeter_altitude;
                combined_location.x_accel = valid_accel.x_accel;
                combined_location.y_accel = valid_accel.y_accel;
                combined_location.z_accel = valid_accel.z_accel;

                return Some(combined_location);
            }
        }

        return None;
    }

    pub fn render_altitude_graph(&self) -> Image {
        let mut pixel_buffer = SharedPixelBuffer::new(1000, 600);
        let size = (pixel_buffer.width() as u32, pixel_buffer.height() as u32);

        let backend = BitMapBackend::with_buffer(pixel_buffer.make_mut_bytes(), size);

        let root = backend.into_drawing_area();

        root.fill(&WHITE).expect("error filling drawing area");

        let mut chart = ChartBuilder::on(&root)
            .build_cartesian_2d(0..self.location_history.len(), 0..1000)
            .expect("error building chart");

        chart
            .configure_mesh()
            .x_labels(20)
            .y_labels(20)
            .y_desc("Altitude (m)")
            .x_desc("Time (s)")
            .axis_desc_style(("sans-serif", 26))
            .draw()
            .expect("error drawing chart");

        // Get 60 most recent data points
        let gps_selection = self.location_history.iter().filter(
            |location| location.gps_altitude.is_some()
        ).rev().take(60);
        
        let alt_selection = self.location_history.iter().filter(
            |location| location.altimeter_altitude.is_some()
        ).rev().take(60);

        // Draw GPS altitude
        chart
            .draw_series(LineSeries::new(
                gps_selection
                    .enumerate()
                    .map(|(index, location)| (index, location.gps_altitude.unwrap() as i32)),
                &Palette99::pick(0),
            ))
            .expect("error drawing series")
            .label("GPS Altitude")
            .legend(move |(x, y)| {
                Rectangle::new([(x - 5, y - 5), (x + 5, y + 5)], &Palette99::pick(0))
            });

        // Draw Altimeter altitude
        chart
            .draw_series(LineSeries::new(
                alt_selection
                    .enumerate()
                    .map(|(index, location)| (index, location.altimeter_altitude.unwrap() as i32)),
                &Palette99::pick(1),
            ))
            .expect("error drawing series")
            .label("Altimeter Altitude")
            .legend(move |(x, y)| {
                Rectangle::new([(x - 5, y - 5), (x + 5, y + 5)], &Palette99::pick(1))
            });

        chart
            .configure_series_labels()
            .background_style(&WHITE.mix(0.8))
            .border_style(&BLACK)
            .draw()
            .expect("error drawing series labels");

        root.present().expect("error presenting");
        drop(chart);
        drop(root);

        sixtyfps::Image::from_rgb8(pixel_buffer)
    }
}

fn find_receiver() -> Option<serialport::SerialPortInfo> {
    // First, get a list of all serial ports available on the system
    let avail_ports = serialport::available_ports().unwrap();

    // Iterate over the available ports and find the first one that matches the
    // serial port name we're looking for
    for port in avail_ports {
        let port_info = format!("{:?}", port.port_type).as_str().to_lowercase();
        if port_info.contains("usb serial device") {
            println!("Found a receiver! {}", port.port_name);
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

lazy_static! {
    static ref GLOBAL_STATE: Arc<Mutex<GlobalState>> = Arc::new(Mutex::new(GlobalState::new()));
}

/// Continuously read from the serial port and parse the data into a Location
/// If the serial port is lost, loop until receiver is found again
fn handle_serial_data(state_clone: Arc<Mutex<GlobalState>>) {
    // Set log file name to date and log file constant
    let file_name = format!("{}-{}",
        chrono::Local::now().format("%Y-%m-%d-%H-%M-%S"),
        LOG_FILE_NAME);

    // Open log file
    let file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(file_name)
        .expect("error opening log file");

    // Create a writer for the log file
    let mut writer = std::io::BufWriter::new(file);

    loop {
        let mut state = state_clone.lock().unwrap();
        state.has_receiver = false;
        drop(state);
        // First, find the serial port
        let port_info = find_receiver();

        // If we didn't find a port, then we can't continue
        if port_info.is_none() {
            println!("Waiting for receiver...");
        } else {
            let mut port = serialport::new(port_info.unwrap().port_name, 9600)
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
            let mut state = state_clone.lock().unwrap();
            state.has_receiver = true;
            drop(state);

            loop {
                let state = state_clone.lock().unwrap();
                let mut transmitters = state.transmitters.clone();
                drop(state);

                let mut line = String::new();
                let n = reader.read_line(&mut line);

                if n.is_ok() {
                    log_to_file(&mut writer, &line);
                    println!("{}", line);

                    // Create new location if sync signal matches
                    if line.starts_with("|") && line.len() > 1 {
                        let id = line.split(",").nth(1).unwrap().parse::<u16>();

                        if let Ok(id) = id {
                            if transmitters.clone().contains_key(&id) {
                                transmitters
                                    .get_mut(&id)
                                    .unwrap()
                                    .ping_history
                                    .push(chrono::Utc::now());
                            } else {
                                let new_transmitter = Transmitter {
                                    id: id.clone(),
                                    location_history: vec![],
                                    ping_history: vec![chrono::Utc::now()],
                                    has_gps: true,
                                    has_altimeter: false,
                                    has_accelerometer: false,
                                };
    
                                transmitters.insert(new_transmitter.id, new_transmitter);
                            }
    
                            let new_loc = Location::new_from_line(line.clone());
    
                            if new_loc.is_ok() {
                                let new_loc = new_loc.unwrap();
    
                                transmitters
                                    .get_mut(&id)
                                    .unwrap()
                                    .location_history
                                    .push(new_loc);
                            }
                        } else {
                            println!("Could not parse id from line: {}", line);
                        }

                        
                    }
                }

                let mut state = state_clone.lock().unwrap();
                state.transmitters = transmitters;
                drop(state);
            }
        }
    }
}

fn log_to_file<W: std::io::Write>(writer: &mut std::io::BufWriter<W>, loc: &String) {
    // Use serde to write most recent location to file
    // with the current time
    let now = chrono::Utc::now();
    let mut serde_loc = serde_json::to_string(&loc).unwrap();
    serde_loc.push_str(&format!("{}", now.format("%Y-%m-%d %H:%M:%S")));
    writer.write_all(serde_loc.as_bytes()).unwrap();
    writer.write_all(b"\n").unwrap();

}

fn update_ui_loop(
    ui_handle: sixtyfps::Weak<sixtyfps_generated_AppWindow::AppWindow>,
    state_clone: Arc<Mutex<GlobalState>>,
) {
    loop {
        let state = state_clone.try_lock();

        if state.is_ok() {
            let state = state.unwrap();
            let has_receiver = state.has_receiver.clone();
            let transmitters = state.transmitters.clone();
            let active_rocket_id = state.active_rocket_id.clone();
            drop(state);

            thread::sleep(Duration::from_millis(10));

            ui_handle.clone().upgrade_in_event_loop(move |ui| {
                // Toggle loading screen
                if has_receiver && ui.get_loading_shown() {
                    ui.set_loading_shown(false);
                } else if !has_receiver && !ui.get_loading_shown() {
                    ui.set_loading_shown(true);
                }

                // Show rockets if they exist
                if has_receiver {
                    // List all transmitters
                    let rocket_text_list = transmitters
                        .iter()
                        .map(|(id, transmitter)| {
                            let diff_ms: i64;
                            // Push difference in ms between last location and now
                            if transmitter.ping_history.len() >= 3 {
                                let last_loc = transmitter
                                    .ping_history
                                    .get(transmitter.ping_history.len() - 1)
                                    .unwrap();
                                let second_last_loc = transmitter
                                    .ping_history
                                    .get(transmitter.ping_history.len() - 3)
                                    .unwrap();
                                let diff = last_loc.signed_duration_since(*second_last_loc);
                                diff_ms = diff.num_milliseconds().max(0).min(999999);
                            } else {
                                diff_ms = 0;
                            }

                            Rocket {
                                id: id.clone() as i32,
                                ms: diff_ms as i32,
                                pings: transmitter.ping_history.len() as i32,
                            }
                        })
                        .collect::<Vec<Rocket>>();

                    let vec_model = sixtyfps::VecModel::from_slice(rocket_text_list.as_slice());

                    ui.set_rocketList(vec_model);

                    // If there is an active rocket, show it
                    if transmitters.contains_key(&active_rocket_id) {
                        let transmitter = transmitters.get(&active_rocket_id).unwrap();

                        // Get last pair of locations
                        let last_full_loc = transmitter.get_last_full_loc();

                        // Display full location data
                        if last_full_loc.is_some() {
                            let converted_loc: activeLocation =
                                last_full_loc.unwrap().to_sixtyfps_loc();

                            ui.set_currentLocation(converted_loc);
                        }

                        // Display altitude graph
                        let alt_graph = transmitter.render_altitude_graph();
                        ui.set_altitudePlot(alt_graph);
                    }
                }
            });
        }
    }
}

fn main() {
    // Spawn two threads:
    //  - one to read from the serial port
    //  - one to handle UI events

    // Serial port thread:
    thread::spawn(move || {
        handle_serial_data(Arc::clone(&GLOBAL_STATE));
    });

    let ui = AppWindow::new();

    let ui_handle = ui.as_weak();

    // Callback to change what rocket to view in the UI
    ui.on_changeActiveRocket(move |id| {
        let state = Arc::clone(&GLOBAL_STATE);

        let mut state = state.try_lock();

        if state.is_ok() {
            state.unwrap().active_rocket_id = id as u16;
        }
    });

    // Main loop to receive info from the receiver,
    // including if it's plugged in and new data
    thread::spawn(move || {
        update_ui_loop(ui_handle, Arc::clone(&GLOBAL_STATE));
    });

    ui.run();
}
