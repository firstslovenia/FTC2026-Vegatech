package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.GenericPIDController;

public class Shooter {

	/// Temp constant, the nominal rpm to run the flywheel at
	public static double FLYWHEEL_NOMINAL_RPM = 100.0;

	/// Nominal power to run the pusher at
	public static double PUSHER_NOMINAL_POWER = 0.4;

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

		shooter_power_pid_controller	= new GenericPIDController(callingOpMode, 0.1, 0.0, 0.0, 0.0);
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
		long time_ms = System.currentTimeMillis();
		long ms_elapsed = time_ms - last_position_time_ms;

		if (ms_elapsed < 10) {
			return;
		}

		int position_ticks = hardware.shooterMotor.getCurrentPosition();
		int delta_position_ticks = position_ticks - last_position_ticks;

		boolean are_measurements_ok = last_position_time_ms != 0;

		if (are_measurements_ok) {
			double ticks_per_second = (double) delta_position_ticks / ((double) ms_elapsed / 1000.0);
			double ticks_per_minute = ticks_per_second * 60.0;
			last_rpm = ticks_per_minute / TICKS_PER_REVOLUTION;
		}

		last_position_time_ms = time_ms;
		last_position_ticks = position_ticks;

		if (are_measurements_ok && wanted_flywheel_rpm != 0.0) {
			shooter_power_pid_controller.error = (wanted_flywheel_rpm - last_rpm) / 100.0;
		}

		shooter_power_pid_controller.update();
		double shooter_power = shooter_power_pid_controller.output;
		shooter_power = Math.min(Math.max(shooter_power, -1.0), 1.0);

		if (shooter_power > 0.0 && started_running_time_ms == 0) {
			started_running_time_ms = time_ms;
		}

		last_slow_start_multiplier = (double) (time_ms - started_running_time_ms) / 10000;
		last_slow_start_multiplier = Math.max(0.0, last_slow_start_multiplier);
		last_slow_start_multiplier = Math.min(1.0, last_slow_start_multiplier);

		shooter_power = shooter_power * last_slow_start_multiplier;
		hardware.shooterMotor.setPower(shooter_power);
	}
}
