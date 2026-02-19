package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.GenericPIDController;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

import java.util.Optional;

public class Shooter {

	/// How many encoder counts mean one revolution on the output axle
	static double TICKS_PER_REVOLUTION = 28.0 * 3.0 * (45.0 / 90.0);

    /// How small the RPM error has to be to be in the perfect stable range
    public static double SHOOTER_RPM_STABLE_ERROR_RANGE = 50.0;

    /// How small the RPM error has to be to be in the semi stable range
    public static double SHOOTER_RPM_SEMI_STABLE_ERROR_RANGE = 200.0;

	/// When we started running the thingo
	long started_running_time_ms = 0;

    ///  When we pushed the pusher upwards
    long enabled_pusher_time_ms = 0;
    public static double RESET_SHOOTER_PUSHER_TIME_MS = 300.0;

	OpMode callingOpMode;
	Hardware hardware;

    /// Currently active PID controller
	GenericPIDController shooter_power_pid_controller;

    /// Used to quickly get the flywheel up to speed
    GenericPIDController startup_pid_controller;
    /// Used afterwards to keep a stable RPM
    GenericPIDController stable_power_pid_controller;

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
    public boolean past_startup = false;

    /// When we started being in the semi-stable zone, so we know to switch PID regulators
    public long started_being_stable_ms = 0;

    /// How long we wait while stable before handing off to the second PID regulator
    public static long PID_HANDOFF_TIME_MS = 200;

	/// The last flywheel encoder position we measured
	public SlidingWindow<Integer> last_position_ticks = new SlidingWindow<>(1, 0);

	/// When we measured last_position_ticks
	public SlidingWindow<Long> last_position_time_ms = new SlidingWindow<>(1, 0L);
	/// Our last few rpm measurements - used to average the measurement out
	public SlidingWindow<Double> last_rpm_measurements = new SlidingWindow<>(5, 0.0, Optional.of(-6000.0), Optional.of(6000.0));

	/// Calculates the distance we expect the shooter the hit for the given RPM
	public static double rpm_to_distance_cm(double rpm) {
        return -Math.pow(rpm, 6.0) / 1008000000000000.0 + (73.0 * Math.pow(rpm, 5.0)) / 3360000000000.0 - (799.0 * Math.pow(rpm, 4.0)) / 4032000000.0 + (64751.0 * Math.pow(rpm, 3.0)) / 67200000.0 - (1659589.0 * Math.pow(rpm, 2.0)) / 630000.0 + (32251693.0 * rpm) / 8400.0 - 2330857.0;
	}

	/// Calculates the RPM to run the shooter at for a given distance
    /// Note: dela samo če si dead on, če ne overshoota
	public static double distance_cm_to_rpm(double distance_cm) {
        return (449338737877.0 * Math.pow(distance_cm, 7.0)) / 7016921487387734630400000.0 - (107277877871999.0 * Math.pow(distance_cm, 6)) / 637901953398884966400000.0 + (305060855409557521.0 * Math.pow(distance_cm, 5.0)) / 1754230371846933657600000.0 - (748963544893509427.0 * Math.pow(distance_cm, 4)) / 7973774417486062080000.0 + (1734636674534632459.0 * Math.pow(distance_cm, 3.0)) / 60076382597497728000.0 - (6295965720321880300573.0 * Math.pow(distance_cm, 2.0)) / 1245902252732197200000.0 + (133495302824634134105707.0 * distance_cm) / 285519266251128525000.0 - 2265549862932106704904.0 / 154501767451909375.0;
	}

	public Shooter(OpMode callingOpMode, Hardware hardware) {

        this.hardware = hardware;
        this.callingOpMode = callingOpMode;

		hardware.shooterMotor.setPower(0.0);
		hardware.shooterPusherServo.setPosition(0.0);

        stable_power_pid_controller = new GenericPIDController(callingOpMode,  0.07, 0.0, 0.07, 0.0);
        startup_pid_controller = new GenericPIDController(callingOpMode, 0.4, 0.0, 0.16, 0.0);

        shooter_power_pid_controller = startup_pid_controller;
    }

	public void disable_flywheel() {
		flywheel_enabled = false;
        wanted_flywheel_rpm = 0.0;
        shooting_distance_m = Double.NaN;

		hardware.shooterMotor.setPower(0.0);

        startup_pid_controller.reset();
        stable_power_pid_controller.reset();
        shooter_power_pid_controller = startup_pid_controller;

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

            startup_pid_controller.reset();
            stable_power_pid_controller.reset();
            shooter_power_pid_controller = startup_pid_controller;

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
	public void fire() {
		pusher_enabled = true;
        enabled_pusher_time_ms = System.currentTimeMillis();
		hardware.shooterPusherServo.setPosition(1.0);
	}

    ///  Moves the shooter pusher back into the down position
    public void reset_shooter_pusher() {
        pusher_enabled = false;
        enabled_pusher_time_ms = 0;
        hardware.shooterPusherServo.setPosition(0.0);
    }

	/// Re-measures RPM and runs PID updates
	public void update() {

        long time_ms = System.currentTimeMillis();

        if (pusher_enabled &&
                enabled_pusher_time_ms != 0 &&
                time_ms - enabled_pusher_time_ms > RESET_SHOOTER_PUSHER_TIME_MS) {
            reset_shooter_pusher();
        }

        if (!flywheel_enabled) {
            hardware.shooterMotor.setPower(0.0);
            return;
        }

        double current_rpm = last_rpm_measurements.average().orElse(0.0);
        double rpm_error = wanted_flywheel_rpm - current_rpm;

        if (!past_startup) {

            if (Math.abs(rpm_error) < SHOOTER_RPM_STABLE_ERROR_RANGE) {

                if (started_being_stable_ms != 0) {
                    if (time_ms - started_being_stable_ms >= PID_HANDOFF_TIME_MS) {
                        // Switch PID controllers
                        past_startup = true;
                        startup_pid_controller.reset();
                        stable_power_pid_controller.reset();
                        shooter_power_pid_controller = stable_power_pid_controller;
                    }
                }

                else {
                    started_being_stable_ms = time_ms;
                }

            } else {
                if (started_being_stable_ms != 0) {
                    started_being_stable_ms = 0;
                }
            }
        }

		double last_pos_ticks = last_position_ticks.average().orElse(0.0);
		double last_time_ms = last_position_time_ms.average().orElse(0.0);

		int position_ticks = hardware.shooterMotor.getCurrentPosition();
		double delta_position_ticks = position_ticks - last_pos_ticks;

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

		if (are_measurements_ok && wanted_flywheel_rpm > 0.0) {
			shooter_power_pid_controller.error = rpm_error / 1000.0;
		}

		shooter_power_pid_controller.update();
		double delta_shooter_power = shooter_power_pid_controller.output;

        // Do slightly less if we're stable
        if (Math.abs(rpm_error) < SHOOTER_RPM_STABLE_ERROR_RANGE) {
            delta_shooter_power *= 0.5;
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
