#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::f32::consts::PI;
use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

const MOTOR_COMMAND_ID: u16 = 0x1FF;

const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);

const VOLTAGE_PROTOCOL_MAX: f32 = 25_000.0;

const PREPARE_VOLTAGE_MAX_YAW: f32 = 2_000.0;
const PREPARE_VOLTAGE_MAX_PITCH: f32 = 15_000.0;
const RUNNING_VOLTAGE_MAX: f32 = 15_000.0;

const PREPARE_RPM_MAX: f32 = 150.0;
const PREPARE_RPM_MAX_PITCH: f32 = 300.0;
const RUNNING_RPM_MAX: f32 = 150.0;

const YAW_ZERO_RAD: f32 = 2.69827;
const PITCH_ZERO_RAD: f32 = 1.222;

const YAW_SIGN: f32 = -1.0;
const PITCH_SIGN: f32 = 1.0;

const YAW_RATE: f32 = 3.0;
const PITCH_RATE: f32 = 2.0;

const PITCH_MIN_RAD: f32 = 0.817;
const PITCH_MAX_RAD: f32 = 1.355;

const STICK_DEADZONE: f32 = 0.08;

const FEEDBACK_TIMEOUT_TICKS: u16 = 100;

const PREPARE_TIMEOUT_TICKS: u16 = 8_000;

const SETTLE_TICKS: u16 = 200;
const SETTLE_ANGLE_RAD: f32 = 0.1;
const SETTLE_SPEED_RPM: f32 = 10.0;

const STALL_TICKS: u16 = 1_000;
const STALL_RPM_DEMAND: f32 = 30.0;
const STALL_SPEED_RPM: f32 = 2.0;

const MAX_TEMP: u8 = 125;

const OSC_WINDOW_TICKS: u16 = 1_000;
const OSC_MIN_HZ: f32 = 4.0;
const OSC_HOT_WINDOWS: u16 = 2;
const OSC_NOISE_RAD: f32 = 0.05;
const OSC_JUMP_RAD: f32 = 1.0;

pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

#[derive(Default)]
pub struct Motor {
    pub angle_rad: f32,
    pub speed_rpm: f32,

    pub current_raw: i16,

    pub command_voltage_raw: f32,
    pub temperature: u8,
}

impl Motor {
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) {
        let raw_angle = u16::from_be_bytes([data[0], data[1]]);
        self.angle_rad = (raw_angle as f32) * (PI * 2.0 / 8192.0);
        self.speed_rpm = i16::from_be_bytes([data[2], data[3]]) as f32;
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);
        self.temperature = data[6];
    }
}

fn wrap_pi(mut angle: f32) -> f32 {
    let turn = PI * 2.0;
    while angle > PI {
        angle -= turn;
    }
    while angle < -PI {
        angle += turn;
    }
    angle
}

fn wrap_0_2pi(mut angle: f32) -> f32 {
    let turn = PI * 2.0;
    while angle >= turn {
        angle -= turn;
    }
    while angle < 0.0 {
        angle += turn;
    }
    angle
}

fn joint_angle(raw_rad: f32, zero_rad: f32) -> f32 {
    wrap_pi(raw_rad - zero_rad)
}

fn stick_unit(value: f32) -> f32 {
    if value.abs() < STICK_DEADZONE {
        0.0
    } else {
        value
    }
}

fn note_half_trip(last: &mut f32, peak: &mut f32, trips: &mut u16, error: f32) {
    let step = error - *last;
    if step.abs() > OSC_JUMP_RAD {
        *last = error;
        *peak = 0.0;
        return;
    }
    if error.abs() > *peak {
        *peak = error.abs();
    }
    let crossed = (*last > 0.0 && error < 0.0) || (*last < 0.0 && error > 0.0);
    if crossed && *peak >= OSC_NOISE_RAD {
        *trips = trips.saturating_add(1);
        *peak = 0.0;
    }
    *last = error;
}

fn axis_is_stuck(target_rpm: f32, voltage: f32, speed_rpm: f32, max_out: f32) -> bool {
    target_rpm.abs() > STALL_RPM_DEMAND
        && max_out > 0.0
        && voltage.abs() > 0.6 * max_out
        && speed_rpm.abs() < STALL_SPEED_RPM
}

#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub max_out: f32,
    pub max_integral: f32,
    integral: f32,
    prev_measured: f32,
    ready: bool,
}

impl Pid {
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measured = 0.0;
        self.ready = false;
    }
    pub fn reset_pitch(&mut self) {
        self.prev_measured = 0.0;
        self.ready = false;
    }

    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        let error = target - measured;

        self.integral += error * dt_s;
        if self.integral > self.max_integral {
            self.integral = self.max_integral;
        } else if self.integral < -self.max_integral {
            self.integral = -self.max_integral;
        }

        let derivative = if self.ready && dt_s > 0.0 {
            (self.prev_measured - measured) / dt_s
        } else {
            0.0
        };
        self.prev_measured = measured;
        self.ready = true;

        let p_term = self.kp * error;
        let i_term = self.ki * self.integral;
        let d_term = self.kd * derivative;
        let mut output = p_term + i_term + d_term;

        if output > self.max_out {
            output = self.max_out;
        } else if output < -self.max_out {
            output = -self.max_out;
        }
        output
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Waiting,

    Prepare,

    Running,

    Error,
}

#[derive(Default)]
pub struct Axis {
    pub motor: Motor,

    pub target_angle_rad: f32,

    pub target_rpm: f32,

    pub position_pid: Pid,

    pub speed_pid: Pid,
}

pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    pub yaw_feedback_age: u16,
    pub pitch_feedback_age: u16,
    pub prepare_ticks: u16,
    pub settle_ticks: u16,
    pub stall_ticks: u16,

    pub fault: u8,
    pub last_right_switch: u8,

    pub left_was_down: bool,
    pub osc_window_ticks: u16,
    pub yaw_half_trips: u16,
    pub pitch_half_trips: u16,
    pub yaw_osc_hot: u16,
    pub pitch_osc_hot: u16,
    pub yaw_osc_hz: f32,
    pub pitch_osc_hz: f32,
    yaw_last_error: f32,
    pitch_last_error: f32,
    yaw_osc_peak: f32,
    pitch_osc_peak: f32,
}

impl Default for Control {
    fn default() -> Self {
        let mut ctrl = Self {
            state: State::Waiting,
            yaw: Axis::default(),
            pitch: Axis::default(),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,

            yaw_feedback_age: u16::MAX,
            pitch_feedback_age: u16::MAX,
            prepare_ticks: 0,
            settle_ticks: 0,
            stall_ticks: 0,
            fault: 0,
            last_right_switch: 0,
            left_was_down: false,
            osc_window_ticks: 0,
            yaw_half_trips: 0,
            pitch_half_trips: 0,
            yaw_osc_hot: 0,
            pitch_osc_hot: 0,
            yaw_osc_hz: 0.0,
            pitch_osc_hz: 0.0,
            yaw_last_error: 0.0,
            pitch_last_error: 0.0,
            yaw_osc_peak: 0.0,
            pitch_osc_peak: 0.0,
        };

        ctrl.yaw.position_pid.kp = 500.0;
        ctrl.yaw.position_pid.ki = 0.0;
        ctrl.yaw.position_pid.kd = 5.0;
        ctrl.yaw.position_pid.max_out = PREPARE_RPM_MAX;
        ctrl.pitch.position_pid.kp = 250.0;
        ctrl.pitch.position_pid.ki = 16.0;
        ctrl.pitch.position_pid.kd = 0.0;
        ctrl.pitch.position_pid.max_out = PREPARE_RPM_MAX_PITCH;

        ctrl.yaw.speed_pid.kp = 300.0;
        ctrl.yaw.speed_pid.ki = 0.0;
        ctrl.yaw.speed_pid.kd = 0.0;
        ctrl.yaw.speed_pid.max_out = RUNNING_VOLTAGE_MAX;
        ctrl.yaw.speed_pid.max_integral = RUNNING_VOLTAGE_MAX;
        ctrl.pitch.speed_pid.kp = 150.0;
        ctrl.pitch.speed_pid.ki = 30.0;
        ctrl.pitch.speed_pid.kd = 0.0;
        ctrl.pitch.speed_pid.max_out = RUNNING_VOLTAGE_MAX;
        ctrl.pitch.speed_pid.max_integral = RUNNING_VOLTAGE_MAX;

        ctrl
    }
}

impl Control {
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    fn reset_pids(&mut self) {
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset_pitch();
        self.pitch.speed_pid.reset_pitch();
    }

    fn reset_detectors(&mut self) {
        self.settle_ticks = 0;
        self.stall_ticks = 0;
        self.osc_window_ticks = 0;
        self.yaw_half_trips = 0;
        self.pitch_half_trips = 0;
        self.yaw_osc_hot = 0;
        self.pitch_osc_hot = 0;
        self.yaw_osc_hz = 0.0;
        self.pitch_osc_hz = 0.0;
        self.yaw_last_error = 0.0;
        self.pitch_last_error = 0.0;
        self.yaw_osc_peak = 0.0;
        self.pitch_osc_peak = 0.0;
    }

    fn feedback_ok(&self) -> bool {
        self.yaw_feedback_age <= FEEDBACK_TIMEOUT_TICKS
            && self.pitch_feedback_age <= FEEDBACK_TIMEOUT_TICKS
    }

    fn yaw_track_error(&self) -> f32 {
        wrap_pi(
            joint_angle(self.yaw.target_angle_rad, YAW_ZERO_RAD)
                - joint_angle(self.yaw.motor.angle_rad, YAW_ZERO_RAD),
        )
    }

    fn pitch_track_error(&self) -> f32 {
        joint_angle(self.pitch.target_angle_rad, PITCH_ZERO_RAD)
            - joint_angle(self.pitch.motor.angle_rad, PITCH_ZERO_RAD)
    }

    fn take_recover(&mut self) -> bool {
        let Some(rc) = self.remote else {
            return false;
        };
        if rc.left_switch == 2 {
            self.left_was_down = true;
        }
        let recover = self.state == State::Error
            && self.left_was_down
            && rc.left_switch == 1
            && rc.right_switch == 3;
        if rc.left_switch == 1 {
            self.left_was_down = false;
        }
        recover
    }

    fn latch_error(&mut self, code: u8) {
        if self.state != State::Error {
            self.fault = code;
            self.disable_all();
            self.state = State::Error;
        }
    }

    fn go_waiting(&mut self) {
        self.disable_all();
        self.state = State::Waiting;
    }

    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();

        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.transmit(can);
    }

    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(packet) = dm_mc02::REMOTE.try_take() {
            match packet {
                Some(frame) => {
                    if self.remote.is_none() {
                        self.last_right_switch = frame.right_switch;
                        self.left_was_down = frame.left_switch == 2;
                    }
                    self.remote = Some(frame);
                    self.remote_silent_ticks = 0;
                }
                None => self.remote = None,
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    pub fn read_feedback(&mut self, can: &mut Can1) {
        self.yaw_feedback_age = self.yaw_feedback_age.saturating_add(1);
        self.pitch_feedback_age = self.pitch_feedback_age.saturating_add(1);
        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;

                    let embedded_can::Id::Standard(id) = *frame.id() else {
                        continue;
                    };
                    let data = frame.data();
                    if data.len() == 8 && id.as_raw() == 0x205 {
                        self.yaw.motor.read(data.try_into().unwrap());
                        self.yaw_feedback_age = 0;
                    } else if data.len() == 8 && id.as_raw() == 0x206 {
                        self.pitch.motor.read(data.try_into().unwrap());
                        self.pitch_feedback_age = 0;
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    pub fn update_mode(&mut self) {
        let right = self.remote.map(|rc| rc.right_switch);
        let enable_edge = right == Some(3) && self.last_right_switch == 2;
        let recover = self.take_recover();
        if let Some(now) = right {
            self.last_right_switch = now;
        }

        let leave_active = right.is_some() && right != Some(3);

        match self.state {
            State::Waiting => {
                if enable_edge && self.feedback_ok() {
                    self.yaw.target_angle_rad = YAW_ZERO_RAD;
                    self.pitch.target_angle_rad = PITCH_ZERO_RAD;
                    self.yaw.target_rpm = 0.0;
                    self.pitch.target_rpm = 0.0;
                    self.reset_pids();
                    self.reset_detectors();
                    self.prepare_ticks = 0;
                    self.fault = 0;
                    self.state = State::Prepare;
                }
            }
            State::Prepare => {
                if leave_active {
                    self.go_waiting();
                    return;
                }
                if !self.feedback_ok() {
                    rtt_target::rprintln!("feedback timeout, back to Waiting");
                    self.go_waiting();
                    return;
                }
                self.prepare_ticks = self.prepare_ticks.saturating_add(1);
                if self.prepare_ticks > PREPARE_TIMEOUT_TICKS {
                    rtt_target::rprintln!(
                        "fault: prepare timeout after {} ticks",
                        self.prepare_ticks
                    );
                    self.latch_error(6);
                    return;
                }
                self.check_faults();
                if self.state == State::Error {
                    return;
                }
                let settled = self.yaw_track_error().abs() < SETTLE_ANGLE_RAD
                    && self.pitch_track_error().abs() < SETTLE_ANGLE_RAD
                    && self.yaw.motor.speed_rpm.abs() < SETTLE_SPEED_RPM
                    && self.pitch.motor.speed_rpm.abs() < SETTLE_SPEED_RPM;
                if settled {
                    self.settle_ticks = self.settle_ticks.saturating_add(1);
                } else {
                    self.settle_ticks = 0;
                }
                if self.settle_ticks >= SETTLE_TICKS {
                    rtt_target::rprintln!("Prepare settled after {} ticks", self.settle_ticks);
                    self.reset_pids();
                    self.reset_detectors();
                    self.state = State::Running;
                }
            }
            State::Running => {
                if leave_active {
                    self.go_waiting();
                    return;
                }
                if !self.feedback_ok() {
                    rtt_target::rprintln!("feedback timeout, back to Waiting");
                    self.go_waiting();
                    return;
                }
                self.check_faults();
            }
            State::Error => {
                self.disable_all();
                let cool = self.yaw.motor.temperature <= MAX_TEMP
                    && self.pitch.motor.temperature <= MAX_TEMP;
                if recover && self.feedback_ok() && cool {
                    let yaw_angle = self.yaw.motor.angle_rad;
                    let pitch_angle = self.pitch.motor.angle_rad;
                    self.yaw.target_angle_rad = yaw_angle;
                    self.pitch.target_angle_rad = pitch_angle;
                    self.yaw.target_rpm = 0.0;
                    self.pitch.target_rpm = 0.0;
                    self.reset_pids();
                    self.reset_detectors();
                    self.fault = 0;
                    self.state = State::Running;
                    rtt_target::rprintln!(
                        "recover yaw angle {:.3} target {:.3}, pitch angle {:.3} target {:.3}, pids reset",
                        yaw_angle,
                        self.yaw.target_angle_rad,
                        pitch_angle,
                        self.pitch.target_angle_rad,
                    );
                }
            }
        }
    }

    fn check_faults(&mut self) {
        let yaw_stuck = axis_is_stuck(
            self.yaw.target_rpm,
            self.yaw.motor.command_voltage_raw,
            self.yaw.motor.speed_rpm,
            self.yaw.speed_pid.max_out,
        );
        let pitch_stuck = axis_is_stuck(
            self.pitch.target_rpm,
            self.pitch.motor.command_voltage_raw,
            self.pitch.motor.speed_rpm,
            self.pitch.speed_pid.max_out,
        );
        if yaw_stuck || pitch_stuck {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
        } else {
            self.stall_ticks = 0;
        }

        let yaw_error = self.yaw_track_error();
        let pitch_error = self.pitch_track_error();
        note_half_trip(
            &mut self.yaw_last_error,
            &mut self.yaw_osc_peak,
            &mut self.yaw_half_trips,
            yaw_error,
        );
        note_half_trip(
            &mut self.pitch_last_error,
            &mut self.pitch_osc_peak,
            &mut self.pitch_half_trips,
            pitch_error,
        );
        self.osc_window_ticks = self.osc_window_ticks.saturating_add(1);
        if self.osc_window_ticks >= OSC_WINDOW_TICKS {
            self.yaw_osc_hz = (self.yaw_half_trips / 2) as f32;
            self.pitch_osc_hz = (self.pitch_half_trips / 2) as f32;
            if self.yaw_osc_hz >= OSC_MIN_HZ {
                self.yaw_osc_hot = self.yaw_osc_hot.saturating_add(1);
            } else {
                self.yaw_osc_hot = 0;
            }
            if self.pitch_osc_hz >= OSC_MIN_HZ {
                self.pitch_osc_hot = self.pitch_osc_hot.saturating_add(1);
            } else {
                self.pitch_osc_hot = 0;
            }
            self.osc_window_ticks = 0;
            self.yaw_half_trips = 0;
            self.pitch_half_trips = 0;
            self.yaw_osc_peak = 0.0;
            self.pitch_osc_peak = 0.0;
        }

        if self.stall_ticks >= STALL_TICKS && self.state != State::Prepare{
            if yaw_stuck {
                rtt_target::rprintln!("fault: yaw stall");
                self.latch_error(1);
            } else {
                rtt_target::rprintln!("fault: pitch stall");
                self.latch_error(2);
            }
            return;
        }
        if self.yaw_osc_hot >= OSC_HOT_WINDOWS {
            rtt_target::rprintln!("fault: yaw oscillation {:.1} Hz", self.yaw_osc_hz);
            self.latch_error(3);
            return;
        }
        if self.pitch_osc_hot >= OSC_HOT_WINDOWS {
            rtt_target::rprintln!("fault: pitch oscillation {:.1} Hz", self.pitch_osc_hz);
            self.latch_error(4);
            return;
        }
        if self.yaw.motor.temperature > MAX_TEMP || self.pitch.motor.temperature > MAX_TEMP {
            rtt_target::rprintln!(
                "fault: overheat yaw {} C pitch {} C",
                self.yaw.motor.temperature,
                self.pitch.motor.temperature,
            );
            self.latch_error(5);
        }
    }

    pub fn update(&mut self) {
        let dt_s = 0.001;
        match self.state {
            State::Waiting | State::Error => {
                self.disable_all();
            }
            State::Prepare => {
                self.yaw.position_pid.max_out = PREPARE_RPM_MAX;
                self.pitch.position_pid.max_out = PREPARE_RPM_MAX_PITCH;
                self.yaw.speed_pid.max_out = PREPARE_VOLTAGE_MAX_YAW;
                self.pitch.speed_pid.max_out = PREPARE_VOLTAGE_MAX_PITCH;
                self.drive_axes(dt_s);
            }
            State::Running => {
                self.yaw.position_pid.max_out = RUNNING_RPM_MAX;
                self.pitch.position_pid.max_out = RUNNING_RPM_MAX;
                self.yaw.speed_pid.max_out = RUNNING_VOLTAGE_MAX;
                self.pitch.speed_pid.max_out = RUNNING_VOLTAGE_MAX;
                if let Some(rc) = self.remote {
                    let yaw_in = stick_unit(rc.channels[2] as f32 / 660.0);
                    let pitch_in = stick_unit(rc.channels[3] as f32 / 660.0);
                    let yaw_joint = wrap_pi(
                        joint_angle(self.yaw.target_angle_rad, YAW_ZERO_RAD)
                            + YAW_SIGN * yaw_in * YAW_RATE * dt_s,
                    );
                    self.yaw.target_angle_rad = wrap_0_2pi(yaw_joint + YAW_ZERO_RAD);
                    self.pitch.target_angle_rad = (self.pitch.target_angle_rad
                        + PITCH_SIGN * pitch_in * PITCH_RATE * dt_s)
                        .clamp(PITCH_MIN_RAD, PITCH_MAX_RAD);
                }
                self.drive_axes(dt_s);
            }
        }
    }

    fn drive_axes(&mut self, dt_s: f32) {
        let yaw_measured = joint_angle(self.yaw.motor.angle_rad, YAW_ZERO_RAD);
        let yaw_error = self.yaw_track_error();

        self.yaw.target_rpm =
            self.yaw
                .position_pid
                .calculate(yaw_measured + yaw_error, yaw_measured, dt_s);
        self.yaw.motor.command_voltage_raw =
            self.yaw
                .speed_pid
                .calculate(self.yaw.target_rpm, self.yaw.motor.speed_rpm, dt_s);

        let pitch_measured = joint_angle(self.pitch.motor.angle_rad, PITCH_ZERO_RAD);
        let pitch_error = self.pitch_track_error();
        self.pitch.target_rpm =
            self.pitch
                .position_pid
                .calculate(pitch_measured + pitch_error, pitch_measured, dt_s);
        self.pitch.motor.command_voltage_raw = self.pitch
                .speed_pid
                .calculate(self.pitch.target_rpm, self.pitch.motor.speed_rpm, dt_s);
    }

    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        let yaw_bytes = voltage_bytes(self.yaw.motor.command_voltage_raw);
        let pitch_bytes = voltage_bytes(self.pitch.motor.command_voltage_raw);
        data[0] = yaw_bytes[0];
        data[1] = yaw_bytes[1];
        data[2] = pitch_bytes[0];
        data[3] = pitch_bytes[1];
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

fn voltage_bytes(command: f32) -> [u8; 2] {
    let raw = command.clamp(-VOLTAGE_PROTOCOL_MAX, VOLTAGE_PROTOCOL_MAX) as i16;
    raw.to_be_bytes()
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    rtt_target::rprintln!(
        "voltage command 0x1FF; measure YAW_ZERO_RAD and PITCH_ZERO_RAD before enable"
    );
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut shown = control.state;

    let mut log_ticks = 0u16;
    loop {
        ticker.next().await;

        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        if control.state != shown {
            rtt_target::rprintln!("state {:?} -> {:?}", shown, control.state);
            shown = control.state;
        }
        log_ticks = log_ticks.saturating_add(1);
        if log_ticks >= 200 {
            log_ticks = 0;
            rtt_target::rprintln!(
                "state={:?} fault={} settle={} stall={} yaw_hz={:.1} pitch_hz={:.1}",
                control.state,
                control.fault,
                control.settle_ticks,
                control.stall_ticks,
                control.yaw_osc_hz,
                control.pitch_osc_hz,
            );
            rtt_target::rprintln!(
                "yaw: angle={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.0}",
                control.yaw.motor.angle_rad,
                control.yaw.target_angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_voltage_raw,
            );
            rtt_target::rprintln!(
                "pitch: angle={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.0}",
                control.pitch.motor.angle_rad,
                control.pitch.target_angle_rad,
                control.pitch.motor.speed_rpm,
                control.pitch.target_rpm,
                control.pitch.motor.command_voltage_raw,
            );
        }
        watchdog.pet();
    }
}
