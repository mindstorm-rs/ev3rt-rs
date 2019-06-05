#![no_std]

#[no_mangle]
pub extern "C" fn abort() -> ! {
    motor_stop(MotorPort::A, false);
    motor_stop(MotorPort::B, false);
    motor_stop(MotorPort::C, false);
    motor_stop(MotorPort::D, false);
    sensor_config(SensorPort::S1, SensorType::NONE);
    sensor_config(SensorPort::S2, SensorType::NONE);
    sensor_config(SensorPort::S3, SensorType::NONE);
    sensor_config(SensorPort::S4, SensorType::NONE);
    unsafe { ev3_exit_task() }
    loop {}
}

use core::panic::PanicInfo;
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    abort()
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum ER {
    OK = 0,
    SYS = -5,
    NOSPT = -9,
    RSFN = -10,
    RSATR = -11,
    PAR = -17,
    ID = -18,
    CTX = -25,
    MACV = -26,
    OACV = -27,
    ILUSE = -28,
    NOMEM = -33,
    NOID = -34,
    NORES = -35,
    OBJ = -41,
    NOEXS = -42,
    QOVR = -43,
    RLWAI = -49,
    TMOUT = -50,
    DLT = -51,
    CLS = -52,
    WBLK = -57,
    BOVR = -58,
}
pub type ErUint = i32;

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum LedColor {
    OFF = 0,
    RED = 1,
    GREEN = 2,
    ORANGE = 3,
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum Button {
    LEFT = 0,
    RIGHT = 1,
    UP = 2,
    DOWN = 3,
    ENTER = 4,
    BACK = 5,
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum SerialPort {
    DEFAULT = 0,
    UART = 1,
    BLUETOOTH = 2,
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum LcdFont {
    SMALL = 0,
    MEDIUM = 1,
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum LcdColor {
    WHITE = 0,
    BLACK = 1,
}

pub const LCD_WIDTH: i32 = 178;
pub const LCD_HEIGHT: i32 = 128;

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum MotorPort {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum MotorType {
    NONE = 0,
    MEDIUM = 1,
    LARGE = 2,
    UNDEGULATED = 3,
}
impl From<i32> for MotorType {
    fn from(t: i32) -> MotorType {
        match t {
            1 => MotorType::MEDIUM,
            2 => MotorType::LARGE,
            3 => MotorType::UNDEGULATED,
            _ => MotorType::NONE,
        }
    }
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum SensorPort {
    S1 = 0,
    S2 = 1,
    S3 = 2,
    S4 = 3,
}

#[repr(i32)]
#[derive(Clone, Copy, PartialEq)]
pub enum SensorType {
    NONE = 0,
    ULTRASONIC = 1,
    GYRO = 2,
    TOUCH = 3,
    COLOR = 4,
    INFRARED = 5,
    HtNxtACCEL = 6,
    HtNxtCOLOR = 7,
    NxtTEMP = 8,
}
impl From<i32> for SensorType {
    fn from(t: i32) -> SensorType {
        match t {
            1 => SensorType::ULTRASONIC,
            2 => SensorType::GYRO,
            3 => SensorType::TOUCH,
            4 => SensorType::COLOR,
            5 => SensorType::INFRARED,
            6 => SensorType::HtNxtACCEL,
            7 => SensorType::HtNxtCOLOR,
            8 => SensorType::NxtTEMP,
            _ => SensorType::NONE,
        }
    }
}

#[repr(i32)]
pub enum SensorColorCode {
    NONE = 0,
    BLACK = 1,
    BLUE = 2,
    GREEN = 3,
    YELLOW = 4,
    RED = 5,
    WHITE = 6,
    BROWN = 7,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct RgbRaw {
    ///!< \~English Red value   \~Japanese 赤
    pub r: u16,
    ///!< \~English Green value \~Japanese 緑
    pub g: u16,
    ///!< \~English Blue value  \~Japanese 青
    pub b: u16,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct IrSeek {
    ///!< \~English Heading  for channels 1-4 (-25 to 25)           \~Japanese 全て（４つ）のチャンネルの方位（-25～25）
    pub heading: [i8; 4usize],
    ///!< \~English Distance for channels 1-4 (-128 and 0 to 100)   \~Japanese 全て（４つ）のチャンネルの距離（0〜100または-128）
    pub distance: [i8; 4usize],
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct IrRemote {
    ///!< \~English IR Remote controller data for channels 1-4   \~Japanese 全て（４つ）のチャンネルのボタン入力のパタン
    pub channel: [u8; 4usize],
}

pub type TMO = i32;
pub type RELTIM = u32;
pub type SYSTIM = i64;
pub type SYSUTM = u64;
pub type BoolT = i8;

extern "C" {
    fn ev3_exit_task() -> ();
    fn ev3_get_utm(p_sysutm: &mut SYSUTM) -> ER;
    fn ev3_sleep(ms: i32) -> ER;

    fn ev3_battery_current_mA() -> i32;
    fn ev3_battery_voltage_mV() -> i32;
    fn ev3_button_is_pressed(button: Button) -> BoolT;

    // fn ev3_serial_open_file(port: serial_port_t) -> *mut FILE;
    // fn ev3_bluetooth_is_connected() -> BoolT;

    fn ev3_lcd_set_font(font: LcdFont) -> ER;
    fn ev3_font_get_size(font: LcdFont, width: *mut i32, height: *mut i32) -> ER;
    fn ev3_lcd_draw_string(str: *const u8, x: i32, y: i32) -> ER;
    fn ev3_lcd_draw_line(x0: i32, y0: i32, x1: i32, y1: i32) -> ER;
    fn ev3_lcd_fill_rect(x: i32, y: i32, w: i32, h: i32, color: LcdColor) -> ER;
    fn ev3_motor_config(port: MotorPort, mt: MotorType) -> ER;
    fn ev3_motor_get_type(port: MotorPort) -> ErUint;
    fn ev3_motor_get_counts(port: MotorPort) -> i32;
    fn ev3_motor_reset_counts(port: MotorPort) -> ER;
    fn ev3_motor_set_power(port: MotorPort, power: i32) -> ER;
    fn ev3_motor_get_power(port: MotorPort) -> i32;
    fn ev3_motor_stop(port: MotorPort, brake: BoolT) -> ER;

    fn ev3_sensor_config(port: SensorPort, st: SensorType) -> ER;
    fn ev3_sensor_get_type(port: SensorPort) -> ErUint;
    fn ev3_color_sensor_get_color(port: SensorPort) -> SensorColorCode;
    fn ev3_color_sensor_get_reflect(port: SensorPort) -> u8;
    fn ev3_color_sensor_get_ambient(port: SensorPort) -> u8;
    fn ev3_color_sensor_get_rgb_raw(port: SensorPort, val: *mut RgbRaw);
    fn ev3_gyro_sensor_get_angle(port: SensorPort) -> i16;
    fn ev3_gyro_sensor_get_rate(port: SensorPort) -> i16;
    fn ev3_gyro_sensor_reset(port: SensorPort) -> ER;
    fn ev3_ultrasonic_sensor_get_distance(port: SensorPort) -> i16;
    // fn ev3_ultrasonic_sensor_listen(port: SensorPort) -> BoolT;

    fn ev3_infrared_sensor_get_distance(port: SensorPort) -> u8;
    // fn ev3_infrared_sensor_seek(port: SensorPort) -> IrSeek;
    // fn ev3_infrared_sensor_get_remote(port: SensorPort) -> IrRemote;
    // fn ev3_touch_sensor_is_pressed(port: SensorPort) -> BoolT;
    // fn ht_nxt_accel_sensor_measure(port: SensorPort, axes: *mut i16) -> BoolT;
    // fn ht_nxt_color_sensor_measure_color(port: SensorPort, color: *mut u8) -> BoolT;
    // fn ht_nxt_color_sensor_measure_rgb(port: SensorPort, val: *mut RgbRaw) -> BoolT;
    // fn nxt_temp_sensor_measure(port: SensorPort, temp: *mut f32) -> BoolT;

    // fn ev3_speaker_set_volume(volume: u8) -> ER;
    // fn ev3_speaker_play_tone(frequency: u16, duration: i32) -> ER;
    // fn ev3_speaker_stop() -> ER;

    fn ev3_led_set_color(color: LedColor) -> ER;
}

pub fn get_utm(p_sysutm: &mut SYSUTM) -> ER {
    unsafe { ev3_get_utm(p_sysutm) }
}

pub fn get_utime() -> SYSUTM {
    let mut res: SYSUTM = 0;
    match get_utm(&mut res) {
        ER::OK => res,
        _ => {
            abort();
        }
    }
}

pub fn sleep(ms: i32) -> ER {
    unsafe { ev3_sleep(ms) }
}

pub fn battery_current_ma() -> i32 {
    unsafe { ev3_battery_current_mA() }
}
pub fn battery_voltage_mv() -> i32 {
    unsafe { ev3_battery_voltage_mV() }
}

pub fn button_is_pressed(button: Button) -> bool {
    unsafe {
        if ev3_button_is_pressed(button) != 0 {
            true
        } else {
            false
        }
    }
}

pub fn lcd_set_font(font: LcdFont) -> ER {
    unsafe { ev3_lcd_set_font(font) }
}

pub fn font_get_size(font: LcdFont) -> (i32, i32) {
    let mut w: i32 = 0;
    let mut h: i32 = 0;
    unsafe {
        let er = ev3_font_get_size(font, &mut w, &mut h);
        if let ER::OK = er {
            (w, h)
        } else {
            (0, 0)
        }
    }
}

pub fn lcd_draw_string(s: &str, x: i32, y: i32) -> ER {
    const MAX: usize = 32;
    const BYTES: usize = MAX + 1;
    let mut chars: [u8; BYTES] = [0; BYTES];
    for (i, c) in s.bytes().enumerate() {
        if i >= MAX {
            break;
        }
        chars[i] = c;
        chars[i + 1] = 0;
    }
    unsafe { ev3_lcd_draw_string(&mut (chars[0]), x, y) }
}

pub fn lcd_draw_line(x0: i32, y0: i32, x1: i32, y1: i32) -> ER {
    unsafe { ev3_lcd_draw_line(x0, y0, x1, y1) }
}

pub fn lcd_fill_rect(x: i32, y: i32, w: i32, h: i32, color: LcdColor) -> ER {
    unsafe { ev3_lcd_fill_rect(x, y, w, h, color) }
}

pub fn lcd_clear() -> ER {
    unsafe { ev3_lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, LcdColor::WHITE) }
}

pub fn motor_config(port: MotorPort, mt: MotorType) -> ER {
    unsafe { ev3_motor_config(port, mt) }
}

pub fn motor_get_type(port: MotorPort) -> MotorType {
    unsafe {
        let t = ev3_motor_get_type(port);
        MotorType::from(t)
    }
}

pub fn motor_get_counts(port: MotorPort) -> i32 {
    unsafe { ev3_motor_get_counts(port) }
}

pub fn motor_reset_counts(port: MotorPort) -> ER {
    unsafe { ev3_motor_reset_counts(port) }
}

pub fn motor_set_power(port: MotorPort, power: i32) -> ER {
    unsafe { ev3_motor_set_power(port, power) }
}
pub fn motor_get_power(port: MotorPort) -> i32 {
    unsafe { ev3_motor_get_power(port) }
}
pub fn motor_stop(port: MotorPort, brake: bool) -> ER {
    unsafe { ev3_motor_stop(port, if brake { 1 } else { 0 }) }
}

pub fn sensor_config(port: SensorPort, st: SensorType) -> ER {
    unsafe { ev3_sensor_config(port, st) }
}

pub fn sensor_get_type(port: SensorPort) -> SensorType {
    unsafe {
        let t = ev3_sensor_get_type(port);
        SensorType::from(t)
    }
}

pub fn color_sensor_get_color(port: SensorPort) -> SensorColorCode {
    unsafe { ev3_color_sensor_get_color(port) }
}
pub fn color_sensor_get_reflect(port: SensorPort) -> u8 {
    unsafe { ev3_color_sensor_get_reflect(port) }
}
pub fn color_sensor_get_ambient(port: SensorPort) -> u8 {
    unsafe { ev3_color_sensor_get_ambient(port) }
}
pub fn color_sensor_get_rgb(port: SensorPort) -> RgbRaw {
    let mut r = RgbRaw { r: 0, g: 0, b: 0 };
    unsafe {
        ev3_color_sensor_get_rgb_raw(port, &mut r);
    }
    r
}
pub fn gyro_sensor_get_angle(port: SensorPort) -> i16 {
    unsafe { ev3_gyro_sensor_get_angle(port) }
}
pub fn gyro_sensor_get_rate(port: SensorPort) -> i16 {
    unsafe { ev3_gyro_sensor_get_rate(port) }
}
pub fn gyro_sensor_reset(port: SensorPort) -> ER {
    unsafe { ev3_gyro_sensor_reset(port) }
}

pub fn ultrasonic_sensor_get_distance(port: SensorPort) -> i16 {
    unsafe { ev3_ultrasonic_sensor_get_distance(port) }
}
// fn ev3_ultrasonic_sensor_listen(port: SensorPort) -> BoolT;

pub fn infrared_sensor_get_distance(port: SensorPort) -> u8 {
    unsafe { ev3_infrared_sensor_get_distance(port) }
}

// fn ev3_infrared_sensor_seek(port: SensorPort) -> IrSeek;
// fn ev3_infrared_sensor_get_remote(port: SensorPort) -> IrRemote;
// fn ev3_touch_sensor_is_pressed(port: SensorPort) -> BoolT;
// fn ht_nxt_accel_sensor_measure(port: SensorPort, axes: *mut i16) -> BoolT;
// fn ht_nxt_color_sensor_measure_color(port: SensorPort, color: *mut u8) -> BoolT;
// fn ht_nxt_color_sensor_measure_rgb(port: SensorPort, val: *mut rgb_raw_t) -> BoolT;
// fn nxt_temp_sensor_measure(port: SensorPort, temp: *mut f32) -> BoolT;

// fn ev3_speaker_set_volume(volume: u8) -> ER;
// fn ev3_speaker_play_tone(frequency: u16, duration: i32) -> ER;
// fn ev3_speaker_stop() -> ER;

pub fn led_set_color(color: LedColor) -> ER {
    unsafe { ev3_led_set_color(color) }
}
