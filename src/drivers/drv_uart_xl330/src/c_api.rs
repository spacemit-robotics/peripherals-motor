use std::sync::Mutex;
use std::time::Duration;
use std::os::raw::{c_char, c_void};
use crate::ReachyMiniMotorController;

// --- C-compatible Structs mirroring motor.h ---

#[repr(C)]
pub struct MotorCmd {
    pub mode: u32,
    pub pos_des: f32,
    pub vel_des: f32,
    pub trq_des: f32,
    pub kp: f32,
    pub kd: f32,
}

#[repr(C)]
pub struct MotorState {
    pub pos: f32,
    pub vel: f32,
    pub trq: f32,
    pub temp: f32,
    pub err: u32,
}

#[repr(C)]
pub struct MotorDev {
    pub name: *const c_char,
    pub ops: *const c_void, // motor_ops pointer
    pub priv_data: *mut c_void, // We store the motor ID (u8) here
}

#[repr(C)]
pub struct ReachyPriv {
    pub id: u8,
    pub dev_path: [c_char; 64],
}

// Global controller instance
static CONTROLLER: Mutex<Option<ReachyMiniMotorController>> = Mutex::new(None);

#[unsafe(no_mangle)]
pub extern "C" fn reachy_motor_init(dev: *mut MotorDev) -> i32 {
    let priv_ptr = unsafe { (*dev).priv_data as *const ReachyPriv };
    let (motor_id, port_path) = unsafe {
        let p = &*priv_ptr;
        let c_str = std::ffi::CStr::from_ptr(p.dev_path.as_ptr());
        (p.id, c_str.to_string_lossy().into_owned())
    };

    let mut lock = CONTROLLER.lock().unwrap();
    if lock.is_none() {
        println!("[Rust Core] Initializing controller on {}", port_path);
        match ReachyMiniMotorController::new(&port_path) {
            Ok(c) => {
                *lock = Some(c);
                // Allow some time for the serial port to stabilize after opening
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                eprintln!("[Rust Core] Error: Failed to open port {}: {:?}", port_path, e);
                return -1;
            }
        }
    }
    
    // Perform motor-specific check if needed
    if let Some(c) = lock.as_mut() {
        match c.ping(motor_id) {
            Ok(true) => {
                // Success
            }
            Ok(false) => {
                eprintln!("[Rust Core] Error: Motor ID {} not responding on {}", motor_id, port_path);
                return -2;
            }
            Err(e) => {
                eprintln!("[Rust Core] Error: Ping failed for ID {} on {}: {:?}", motor_id, port_path, e);
                return -2;
            }
        }

        // Enable torque by default during init
        if let Err(e) = c.enable_torque_on_ids(&[motor_id]) {
            eprintln!("[Rust Core] Error: Failed to enable torque for ID {} on {}: {:?}", motor_id, port_path, e);
            return -3;
        }
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn reachy_motor_set_cmd(dev: *mut MotorDev, cmd: *const MotorCmd) -> i32 {
    let priv_ptr = unsafe { (*dev).priv_data as *const ReachyPriv };
    let motor_id = unsafe { (*priv_ptr).id };
    let cmd = unsafe { &*cmd };
    
    let mut lock = CONTROLLER.lock().unwrap();
    if let Some(c) = lock.as_mut() {
        // Handle mode: MOTOR_MODE_OFF (3 in motor.h) to disable torque
        if cmd.mode == 3 {
             let _ = c.disable_torque_on_ids(&[motor_id]);
             return 0;
        }
        
        // Ensure torque is enabled for other control modes
        // (Position is 0, Vel is 1, Trq is 2)
        
        // We convert rad to the servo's raw resolution (center ~2048)
        let raw_pos = (cmd.pos_des * (2048.0 / std::f32::consts::PI) + 2048.0) as f64;
        
        if c.write_raw_bytes(motor_id, 116, &((raw_pos as u32).to_le_bytes())).is_err() {
            return -1;
        }
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn reachy_motor_get_state(dev: *mut MotorDev, state: *mut MotorState) -> i32 {
    let priv_ptr = unsafe { (*dev).priv_data as *const ReachyPriv };
    let motor_id = unsafe { (*priv_ptr).id };
    let state = unsafe { &mut *state };
    
    let mut lock = CONTROLLER.lock().unwrap();
    if let Some(c) = lock.as_mut() {
        // Read position (Address 132 for XL330)
        match c.read_raw_bytes(motor_id, 132, 4) {
            Ok(data) if data.len() == 4 => {
                let raw_pos = u32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f32;
                state.pos = (raw_pos - 2048.0) * (std::f32::consts::PI / 2048.0);
            }
            _ => return -1,
        }
        
        // Read current (Address 126)
        match c.read_raw_bytes(motor_id, 126, 2) {
            Ok(data) if data.len() == 2 => {
                let raw_cur = i16::from_le_bytes([data[0], data[1]]);
                state.trq = raw_cur as f32 * 0.001; // Rough approximation
            }
            _ => {}
        }
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn reachy_motor_free(dev: *mut MotorDev) {
    if dev.is_null() { return; }
    let priv_ptr = unsafe { (*dev).priv_data as *const ReachyPriv };
    let motor_id = unsafe { (*priv_ptr).id };
    
    let mut lock = CONTROLLER.lock().unwrap();
    if let Some(c) = lock.as_mut() {
        let _ = c.disable_torque_on_ids(&[motor_id]);
    }
    // We don't free the global controller here, just the C-side dev struct if needed
}
