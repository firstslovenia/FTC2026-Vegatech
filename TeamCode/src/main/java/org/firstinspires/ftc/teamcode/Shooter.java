package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.GenericPIDController;

public class Shooter {

	/// Temp constant, the nominal rpm to run the flywheel at
	public static double FLYWHEEL_NOMINAL_RPM = 1000.0;

	/// Nominal power to run the pusher at
	public static double PUSHER_NOMINAL_POWER = 0.4;

	/// Minimum power to actually do anything on the flywheel
	public static double FLYWHEEL_FEED_FORWARD = 0.1;

	/// How long the slow start takes
	public static double FLYWHEEL_SLOW_START_TIME = 3000;

	/// How many encoder counts mean one revolution on the output axle
	static double TICKS_PER_REVOLUTION = 28.0 * 3.0 * (30.0 / 125.0);

	/// When we started running the thingo
	long started_running_time_ms = 0;

	OpMode callingOpMode;
	Hardware hardware;

	GenericPIDController shooter_power_pid_controller;

	/// What RPM we want the flywheel's RPM to be
	public double wanted_flywheel_rpm = 0.0;

	/// Whether we are powering the flywheel currently
	public boolean flywheel_enabled = false;

	/// Whether we are powering the pusher currently
	public boolean pusher_enabled = false;

	/// The last flywheel encoder position we measured
	int last_position_ticks = 0;
	/// When we measured last_position_ticks
	long last_position_time_ms = 0;

	/// The last rpm we measured
	public double last_rpm = 0.0;

	public double last_slow_start_multiplier = 0.0;

	public Shooter(OpMode callingOpMode, Hardware hardware) {
		hardware.shooterMotor.setPower(0.0);
		hardware.shooterPusherMotor.setPower(0.0);

		shooter_power_pid_controller	= new GenericPIDController(callingOpMode, 0.06, 0.0, 0.0, 0.0);
	}

	public void disable_flywheel() {
		flywheel_enabled = false;
		hardware.shooterMotor.setPower(0.0);
		shooter_power_pid_controller.reset();
		started_running_time_ms = 0;
	}

	public void update_flywheel_rpm(double flywheel_rpm) {
		if (flywheel_rpm == 0.0) {
			disable_flywheel();
		}	else {
			wanted_flywheel_rpm = flywheel_rpm;
			flywheel_enabled = true;
		}
	}

	public void update_pusher_power(double pusher_power) {
		pusher_enabled = pusher_power != 0.0;
		hardware.shooterPusherMotor.setPower(pusher_power);
	}

	/// Re-measures RPM and runs PID updates
	public void update() {
		int position_ticks = hardware.shooterMotor.getCurrentPosition();
		int delta_position_ticks = position_ticks - last_position_ticks;

		long time_ms = System.currentTimeMillis();
		long ms_elapsed = time_ms - last_position_time_ms;

		if (last_position_time_ms == 0) {
			last_position_time_ms = time_ms;
			last_position_ticks = position_ticks;
		}

		boolean started_measurements = last_position_time_ms != 0;
		boolean enough_elapsed = (time_ms - last_position_time_ms > 20);
		boolean not_same_reading = (last_position_ticks != position_ticks) || (time_ms - last_position_time_ms > 50);

		boolean are_measurements_ok = started_measurements && enough_elapsed && not_same_reading;

		if (are_measurements_ok) {
			double ticks_per_second = (double) delta_position_ticks * 1000.0 / ms_elapsed;
			double ticks_per_minute = ticks_per_second * 60.0;
			last_rpm = ticks_per_minute / TICKS_PER_REVOLUTION;
			last_position_time_ms = time_ms;
			last_position_ticks = position_ticks;
		}

		if (are_measurements_ok && wanted_flywheel_rpm != 0.0) {
			shooter_power_pid_controller.error = (wanted_flywheel_rpm - last_rpm) / 100.0;
		}

		shooter_power_pid_controller.update();
		double shooter_power = shooter_power_pid_controller.output;
		shooter_power = Math.min(Math.max(shooter_power, -1.0), 1.0);

		// Map -1 (max descelleration) to 0, to naturally coast along
		if (shooter_power <= 0.0) {
			shooter_power = shooter_power + 1.0;
			shooter_power = shooter_power * FLYWHEEL_FEED_FORWARD;
		} else {
			double power_threshold = 1.0 - FLYWHEEL_FEED_FORWARD;
			shooter_power = shooter_power * power_threshold + FLYWHEEL_FEED_FORWARD;
		}

		shooter_power = Math.min(Math.max(shooter_power, -1.0), 1.0);


		if (shooter_power > 0.0 && started_running_time_ms == 0) {
			started_running_time_ms = time_ms;
		}

		last_slow_start_multiplier = (double) (time_ms - started_running_time_ms) / FLYWHEEL_SLOW_START_TIME;
		last_slow_start_multiplier = Math.max(0.0, last_slow_start_multiplier);
		last_slow_start_multiplier = Math.min(1.0, last_slow_start_multiplier);

		shooter_power = shooter_power * last_slow_start_multiplier;
		hardware.shooterMotor.setPower(shooter_power);
	}
}
