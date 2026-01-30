package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.GenericPIDController;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

public class Shooter {

	/// Nominal power to run the pusher at
	public static double PUSHER_NOMINAL_POWER = 0.4;

	/// Minimum power to actually do anything on the flywheel
	public static double FLYWHEEL_FEED_FORWARD = 0.033;

	/// How many encoder counts mean one revolution on the output axle
	static double TICKS_PER_REVOLUTION = 28.0 * 3.0 * (45.0 / 90.0);

	/// When we started running the thingo
	long started_running_time_ms = 0;

	OpMode callingOpMode;
	Hardware hardware;

	GenericPIDController shooter_power_pid_controller;

    ///  What distance we are regulating for
    public double shooting_distance_m = Double.NaN;
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

    /// Set to false before we start oscillating (before we ever go above the wanted RPM)
    public boolean started_oscillating = false;

	/// The last flywheel encoder position we measured
	public SlidingWindow<Integer> last_position_ticks = new SlidingWindow<>(10, 0);

	/// When we measured last_position_ticks
	public SlidingWindow<Long> last_position_time_ms = new SlidingWindow<>(10, 0L);

	/// Our last few rpm measurements - used to average the measurement out
	public SlidingWindow<Double> last_rpm_measurements = new SlidingWindow<>(50, 0.0);

	/// Calculates the distance we expect the shooter the hit for the given RPM
	public static double rpm_to_distance_cm(double rpm) {
		return -(1257301.0 * Math.pow(rpm, 5))/3727189786320000000.0 + (144811837 * Math.pow(rpm, 4))/26622784188000000.0 - (1305459600047.0 * Math.pow(rpm ,3))/37271897863200000.0 + (1198650629021.0 * Math.pow(rpm, 2))/10649113675200.0 - (933411826993.0 * rpm)/5176652481.0 + 595604859575.0/5171481.0;
	}

	/// Calculates the RPM to run the shooter at for a given distance
	public static double distance_cm_to_rpm(double distance_cm) {
		return (836237.0 * Math.pow(distance_cm, 5))/45067522500000.0 - (100509133.0 * Math.pow(distance_cm, 4))/4506752250000.0 + (2735851519.0 * Math.pow(distance_cm, 3))/257528700000.0 - (56390675269.0 * Math.pow(distance_cm, 2))/22533761250.0 + (5938811617.0 * distance_cm)/20030010.0 - 3273971284.0/286143.0;
	}

	public Shooter(OpMode callingOpMode, Hardware hardware) {
		hardware.shooterMotor.setPower(0.0);
		hardware.shooterPusherServo.setPosition(0.0);

		//shooter_power_pid_controller = new GenericPIDController(callingOpMode, 0.08, 0.0, 0.015, 0.0);
        // Še kr okej dela 0.1, 0.005, 0.05
        //shooter_power_pid_controller = new GenericPIDController(callingOpMode, 0.2, 0.0, 0.1, 0.0);
        shooter_power_pid_controller = new GenericPIDController(callingOpMode,  0.03, 0.0, 0.002, 0.0);
    }

	public void disable_flywheel() {
		flywheel_enabled = false;
        wanted_flywheel_rpm = 0.0;
        shooting_distance_m = Double.NaN;
		hardware.shooterMotor.setPower(0.0);
		shooter_power_pid_controller.reset();
		started_running_time_ms = 0;
	}

    public void update_rpm_for_distance_m(double distance_m) {
        update_flywheel_rpm(distance_cm_to_rpm(distance_m * 100.0));
        shooting_distance_m = distance_m;
    }

	public void update_flywheel_rpm(double flywheel_rpm) {
		if (flywheel_rpm == 0.0) {
			disable_flywheel();
		}	else {
			wanted_flywheel_rpm = flywheel_rpm;

            if (wanted_flywheel_rpm > 1500) {
                // Preemtively, so the PID is faster
                flywheel_power = 0.4;
                hardware.shooterMotor.setPower(0.4);
            }

			flywheel_enabled = true;
		}
	}

    ///  Returns how far off we are from the wanted RPM.
    ///
    /// Returs NaN if the shooter is not currently enabled
    public double get_rpm_error() {
        if (!flywheel_enabled) {
            return Double.NaN;
        } else {
            return Math.abs(wanted_flywheel_rpm - last_rpm_measurements.average().orElse(0.0));
        }
    }

    ///  Moves the shooter pusher into the up position, feeding a ball into the flywheel
	public void feed_ball_into_shooter() {
		pusher_enabled = true;
		hardware.shooterPusherServo.setPosition(1.0);
	}

    ///  Moves the shooter pusher back into the down position
    public void reset_shooter_pusher() {
        pusher_enabled = false;
        hardware.shooterPusherServo.setPosition(0.0);
    }

	/// Re-measures RPM and runs PID updates
	public void update() {

        if (!flywheel_enabled) {
            hardware.shooterMotor.setPower(0.0);
            return;
        }

        double current_rpm = last_rpm_measurements.average().orElse(0.0);

        // Slight crutch
        if (!started_oscillating && current_rpm > wanted_flywheel_rpm) {
            started_oscillating = true;
            shooter_power_pid_controller.reset();
        }

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

        // Magic!! Amazing!! Beatiful
        if (Math.abs(delta_shooter_power) < 75.0) {
            delta_shooter_power *= 0.75;
        }

		flywheel_power = flywheel_power + (delta_shooter_power * 36.0 / 1000.0);
		flywheel_power = Math.min(Math.max(flywheel_power, -1.0), 1.0);

		flywheel_power = Math.min(Math.max(flywheel_power, -1.0), 1.0);

		if (flywheel_power > 0.0 && started_running_time_ms == 0) {
			started_running_time_ms = time_ms;
		}

		if (!dry_run) {
			hardware.shooterMotor.setPower(flywheel_power);
		}
	}
}
