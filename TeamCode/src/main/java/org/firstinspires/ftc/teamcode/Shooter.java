package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.GenericPIDController;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

import java.util.Optional;

public class Shooter {

    /// How many encoder counts mean one revolution on the output axle
    static double TICKS_PER_REVOLUTION = 28.0 * 3.0 * (45.0 / 90.0);

    /// A constant that gives a rough estimation of needed power / wanted RPM
    static double POWER_PER_RPM_COEFF = 0.893 / 3000.0;

    /// How small the RPM error has to be to be in the perfect stable range
    public static double SHOOTER_RPM_STABLE_ERROR_RANGE = 50.0;

    /// How small the RPM error has to be to be in the semi stable range
    public static double SHOOTER_RPM_SEMI_STABLE_ERROR_RANGE = 200.0;

    /// When we started running the thingo
    long started_running_time_ms = 0;

    ///  When we pushed the pusher upwards
    long enabled_pusher_time_ms = 0;
    public static double RESET_SHOOTER_PUSHER_TIME_MS = 400.0;

    /// Baseline voltage to apply power for
    public static double NOMINAL_VOLTAGE_V = 12.0;

    OpMode callingOpMode;
    Hardware hardware;
    Spindexer spindexer;

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

    /// When we started being in the stable zone, so we know to switch PID regulators or fire balls
    public long started_being_stable_ms = 0;

    /// How long we wait while stable before handing off to the second PID regulator
    public static long PID_HANDOFF_TIME_MS = 500;

    /// How long we have to be stable for to shoot
    public static long STABLE_SHOOTING_TIME_MS = 300;

    /// The last flywheel encoder position we measured
    public SlidingWindow<Integer> last_position_ticks = new SlidingWindow<>(1, 0);

    /// When we measured last_position_ticks
    public SlidingWindow<Long> last_position_time_ms = new SlidingWindow<>(1, 0L);
    /// Our last few rpm measurements - used to average the measurement out
    public SlidingWindow<Double> last_rpm_measurements = new SlidingWindow<>(5, 0.0, Optional.of(-6000.0), Optional.of(6000.0));

    /// Calculates the distance we expect the shooter the hit for the given RPM
    public static double rpm_to_distance_cm(double rpm) {
        return (17.0 * Math.pow(rpm,3))/14400000.0 - (1609.0 * Math.pow(rpm, 2.0))/144000.0 + (5099.0 * rpm)/144.0 - 112025.0/3.0;
    }

    /// Calculates the RPM to run the shooter at for a given distance
    /// Note: dela samo če si dead on, če ne overshoota
    public static double distance_cm_to_rpm(double distance_cm) {
        return -(28811.0 * Math.pow(distance_cm, 4.0))/66621555000.0 + (39493.0 * Math.pow(distance_cm, 3.0))/222071850.0 + (37968083.0 * Math.pow(distance_cm,2.0))/2664862200.0 - (41816611.0 * distance_cm)/4934930.0 + 456799660.0/134589.0;
    }

    public Shooter(OpMode callingOpMode, Hardware hardware, Spindexer spindexer) {

        this.hardware = hardware;
        this.callingOpMode = callingOpMode;
        this.spindexer = spindexer;

        hardware.shooterMotor.setPower(0.0);
        hardware.shooterPusherServo.setPosition(0.0);

        stable_power_pid_controller = new GenericPIDController(callingOpMode, 0.25, 0.0, 0.3, 0.0);
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

    /// Sets the flywheel to a given power level for the nominal voltage
    public void set_flywheel_power(double power) {
        double voltage_v = callingOpMode.hardwareMap.voltageSensor.iterator().next().getVoltage();
        double voltage_coeff = NOMINAL_VOLTAGE_V / voltage_v;

        double compensated_power = power * voltage_coeff;
        compensated_power = Math.min(Math.max(compensated_power, -1.0), 1.0);
        flywheel_power = compensated_power / voltage_coeff;

        hardware.shooterMotor.setPower(compensated_power);
    }

    public void update_rpm_for_distance_m(double distance_m) {
        update_flywheel_rpm(distance_cm_to_rpm(distance_m * 100.0));
        shooting_distance_m = distance_m;
    }

    public void update_flywheel_rpm(double flywheel_rpm) {
        if (flywheel_rpm == 0.0) {
            disable_flywheel();
        } else {
            wanted_flywheel_rpm = flywheel_rpm;

            if (wanted_flywheel_rpm > 1500) {
                // Preemtively, so the PID is faster
                set_flywheel_power(POWER_PER_RPM_COEFF * flywheel_rpm);
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

        started_being_stable_ms = 0;
    }

    ///  Moves the shooter pusher back into the down position
    public void reset_shooter_pusher() {
        pusher_enabled = false;
        enabled_pusher_time_ms = 0;
        hardware.shooterPusherServo.setPosition(0.0);
    }

    /// Returns true when we're okay to fire
    public boolean is_ready_to_fire() {

        boolean flywheel_ready = flywheel_enabled && Math.abs(get_rpm_error()) < SHOOTER_RPM_STABLE_ERROR_RANGE;
        boolean flywheel_stable = started_being_stable_ms != 0 && System.currentTimeMillis() - started_being_stable_ms > STABLE_SHOOTING_TIME_MS;
        boolean spindexer_not_busy = !spindexer.is_motor_busy() && spindexer.can_move();

        return flywheel_ready && flywheel_stable && spindexer_not_busy;
    }

    /// Re-measures RPM and runs PID updates
    public void update() {

        long time_ms = System.currentTimeMillis();

        if (pusher_enabled && enabled_pusher_time_ms != 0 && time_ms - enabled_pusher_time_ms > RESET_SHOOTER_PUSHER_TIME_MS) {
            reset_shooter_pusher();
        }

        if (!flywheel_enabled) {
            hardware.shooterMotor.setPower(0.0);
            return;
        }

        double current_rpm = last_rpm_measurements.average().orElse(0.0);
        double rpm_error = wanted_flywheel_rpm - current_rpm;

        if (Math.abs(rpm_error) < SHOOTER_RPM_STABLE_ERROR_RANGE) {
            if (started_being_stable_ms == 0) {
                started_being_stable_ms = time_ms;
            }

        } else {
            if (started_being_stable_ms != 0) {
                started_being_stable_ms = 0;
            }
        }

        if (!past_startup && started_being_stable_ms != 0 && time_ms - started_being_stable_ms >= PID_HANDOFF_TIME_MS) {
            // Switch PID controllers
            past_startup = true;
            startup_pid_controller.reset();
            stable_power_pid_controller.reset();
            shooter_power_pid_controller = stable_power_pid_controller;
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

        if (flywheel_power > 0.0 && started_running_time_ms == 0) {
            started_running_time_ms = time_ms;
        }

        if (!dry_run) {
            set_flywheel_power(flywheel_power);
        }
    }
}
