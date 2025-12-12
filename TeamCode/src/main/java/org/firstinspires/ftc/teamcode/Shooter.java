package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.GenericPIDController;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

public class Shooter {

	/// Nominal power to run the pusher at
	public static double PUSHER_NOMINAL_POWER = 0.4;

	/// Minimum power to actually do anything on the flywheel
	public static double FLYWHEEL_FEED_FORWARD = 0.033;

	/// How long the slow start takes
	public static double FLYWHEEL_SLOW_START_TIME = 3000;

	/// Multiplier to apply to negative PID values
	///
	/// - it seems increasing power has a much greater effect than decreasing it,
	///  causing oscillations around the wrong value
	///
	/// This is a bandaid-ish fix
	public static double FLYWHEEL_NEGATIVE_PID_MULTIPLIER = 10.0;

	/// How many encoder counts mean one revolution on the output axle
	static double TICKS_PER_REVOLUTION = 28.0 * 3.0 * (45.0 / 90.0);

	/// When we started running the thingo
	long started_running_time_ms = 0;

	OpMode callingOpMode;
	Hardware hardware;

	GenericPIDController shooter_power_pid_controller;

	/// What RPM we want the flywheel's RPM to be
	public double wanted_flywheel_rpm = 0.0;

	/// The current flywheel's power - what we're regulating
	public double flywheel_power = 0.0;

	/// Whether we are powering the flywheel currently
	public boolean flywheel_enabled = false;

	/// Whether we are powering the pusher currently
	public boolean pusher_enabled = false;

	/// Whether to not actually update motor values
	public boolean dry_run = false;

	/// The last flywheel encoder position we measured
	public SlidingWindow<Integer> last_position_ticks = new SlidingWindow<>(10, 0);

	/// When we measured last_position_ticks
	public SlidingWindow<Long> last_position_time_ms = new SlidingWindow<>(10, 0L);

	/// Our last few rpm measurements - used to average the measurement out
	public SlidingWindow<Double> last_rpm_measurements = new SlidingWindow<>(50, 0.0);

	public double last_slow_start_multiplier = 0.0;

	public Shooter(OpMode callingOpMode, Hardware hardware) {
		hardware.shooterMotor.setPower(0.0);
		hardware.shooterPusherMotor.setPower(0.0);

		shooter_power_pid_controller = new GenericPIDController(callingOpMode, 0.3, 0.0, 0.08, 0.0);
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

		double last_pos_ticks = last_position_ticks.average().orElse(0.0);
		double last_time_ms = last_position_time_ms.average().orElse(0.0);

		int position_ticks = hardware.shooterMotor.getCurrentPosition();
		double delta_position_ticks = position_ticks - last_pos_ticks;

		long time_ms = System.currentTimeMillis();
		double ms_elapsed = time_ms - last_time_ms;

		if (!last_position_time_ms.first().isPresent()) {
			last_position_time_ms.push(time_ms);
			last_position_ticks.push(position_ticks);
		}

		boolean started_measurements = last_position_time_ms.first().isPresent();
		boolean enough_elapsed = (time_ms - last_time_ms > 10);
		boolean not_same_reading = (last_pos_ticks != position_ticks) || (time_ms - last_time_ms > 5);

		boolean are_measurements_ok = started_measurements && enough_elapsed && not_same_reading;

		if (are_measurements_ok) {
			double ticks_per_second = (double) delta_position_ticks * 1000.0 / ms_elapsed;
			double ticks_per_minute = ticks_per_second * 60.0;

			last_rpm_measurements.push(ticks_per_minute / TICKS_PER_REVOLUTION);
			last_position_time_ms.push(time_ms);
			last_position_ticks.push(position_ticks);
		}

		if (are_measurements_ok && wanted_flywheel_rpm != 0.0) {
			shooter_power_pid_controller.error = (wanted_flywheel_rpm - last_rpm_measurements.average().orElse(0.0)) / 1000.0;
		}

		shooter_power_pid_controller.update();
		double delta_shooter_power = shooter_power_pid_controller.output;

		if (delta_shooter_power < 0.0) {
			delta_shooter_power *= FLYWHEEL_NEGATIVE_PID_MULTIPLIER;
		}

		flywheel_power = flywheel_power + (delta_shooter_power * ms_elapsed / 1000.0);
		flywheel_power = Math.min(Math.max(flywheel_power, -1.0), 1.0);

		// Map -1 (max deceleration) to 0, to naturally coast along
		if (flywheel_power <= 0.0) {
			flywheel_power = flywheel_power + 1.0;
			flywheel_power = flywheel_power * FLYWHEEL_FEED_FORWARD;
		} else {
			double power_threshold = 1.0 - FLYWHEEL_FEED_FORWARD;
			flywheel_power = flywheel_power * power_threshold + FLYWHEEL_FEED_FORWARD;
		}

		flywheel_power = Math.min(Math.max(flywheel_power, -1.0), 1.0);

		if (flywheel_power > 0.0 && started_running_time_ms == 0) {
			started_running_time_ms = time_ms;
		}

		last_slow_start_multiplier = (double) (time_ms - started_running_time_ms) / FLYWHEEL_SLOW_START_TIME;
		last_slow_start_multiplier = Math.max(0.0, last_slow_start_multiplier);
		last_slow_start_multiplier = Math.min(1.0, last_slow_start_multiplier);

		flywheel_power = flywheel_power * last_slow_start_multiplier;

		if (!dry_run) {
			hardware.shooterMotor.setPower(flywheel_power);
		}
	}
}
