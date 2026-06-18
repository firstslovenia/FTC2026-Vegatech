package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

import java.util.Optional;

public class ShooterPlusPlus {

    /// How many encoder counts mean one revolution on the output axle
    static double TICKS_PER_REVOLUTION = 28.0;

    /// Tau for our startup regulation
    static double STARTUP_TAU = 0.5;

    // --- Measured constants ------
    // Power to rpm conversion: power is in percentages, rpm just rpm
    // See graph: https://www.wolframalpha.com/input?i=fit+linear&assumption=%7B%22F%22%2C+%22LinearFitCalculator%22%2C+%22data2%22%7D+-%3E%22%7B%7B100.0%2C+4950%7D%2C%7B90.0%2C+4450%7D%2C%7B80.0%2C+4050%7D%2C%7B70.0%2C+3460%7D%2C%7B60.0%2C+2950%7D%2C+%7B50.0%2C+2450%7D%7D%22
    // EEEE: https://www.wolframalpha.com/input?i=fit+linear&assumption=%7B%22F%22%2C+%22LinearFitCalculator%22%2C+%22data2%22%7D+-%3E%22%7B%7B86.0%2C+4500%7D%2C%7B66.0%2C+3500%7D%2C%7B49.0%2C+2500%7D%7D%22
    /// The factor times x in the power to rpm conversion
    static double REGULATOR_POW_TO_RPM_K = 53.94;

    /// The factor times 1 in the power to rpm conversion
    static double REGULATOR_POW_TO_RPM_C = -113.70;

    /// The voltage (under load) used for the power to rpm convesion
    static double REGULATOR_BASE_VOLTAGE = 12.33;

    /// The lowest and highest angles (servo 0.0 and 1.0 respectively)
    static double ANGLE_SERVO_START = 20.0 / 180.0 * Math.PI;
    static double ANGLE_SERVO_END = 38.5 / 180.0 * Math.PI;

    /// Coefficients from the best fit quadratic for distance_factor(angle in rads), where the distance factor is multiplied
    /// times the lowest angle we can shoot
    ///
    /// See [...](https://www.wolframalpha.com/input?i=fit+quadratic&assumption=%7B%22F%22%2C+%22QuadraticFitCalculator%22%2C+%22data2%22%7D+-%3E%22%7B%7B0.349%2C+1.0%7D%2C+%7B0.435%2C+0.943%7D%2C+%7B0.521%2C+0.795%7D%2C+%7B0.607%2C+0.614%7D%2C+%7B0.672%2C+0.400%7D%7D%22)
    /// largest residual: 1.2%, 5.28 cm
    static double DISTANCE_ANGLE_FUNC_A = -4.72322;
    static double DISTANCE_ANGLE_FUNC_B = 2.9753;
    static double DISTANCE_ANGLE_FUNC_C = 0.537631;

    /// Coefficients for the inverse function of the one above.
    ///
    /// See
    static double INVERSE_DIST_ANGLE_FUNC_A = -0.64999;
    static double INVERSE_DIST_ANGLE_FUNC_B = 0.403191;
    static double INVERSE_DIST_ANGLE_FUNC_C = 0.611814;

    // ------------------------------

    /// The max flywheel power we have
    public double flywheel_gain = 1.0;

    /// When we started running the thingo
    long started_running_time_ms = 0;
    OpMode callingOpMode;
    Hardware hardware;
    Spindexer spindexer;

    ///  What distance we are regulating for
    public double shooting_distance_m = Double.NaN;

    /// What RPM we want the flywheel's RPM to be
    public double wanted_flywheel_rpm = 0.0;

    /// The current flywheel's power - what we're regulating
    public double flywheel_power = 0.0;

    /// The relative power of the flywheel, used only for startup
    public double flywheel_rel_power = 0.0;

    /// Whether we are powering the flywheel currently
    public boolean flywheel_enabled = false;

    /// Whether to not actually update motor values
    public boolean dry_run = false;

    /// The last flywheel encoder position we measured
    public SlidingWindow<Integer> last_b_position_ticks = new SlidingWindow<>(1, 0);
    public SlidingWindow<Integer> last_a_position_ticks = new SlidingWindow<>(1, 0);

    /// When we measured last_position_ticks
    public SlidingWindow<Long> last_position_time_ms = new SlidingWindow<>(1, 0L);
    /// Our last few rpm measurements - used to average the measurement out
    public SlidingWindow<Double> last_a_rpm_measurements = new SlidingWindow<>(5, 0.0, Optional.of(-6000.0), Optional.of(6000.0));
    public SlidingWindow<Double> last_b_rpm_measurements = new SlidingWindow<>(5, 0.0, Optional.of(-6000.0), Optional.of(6000.0));

    public ShooterPlusPlus(OpMode callingOpMode, Hardware hardware, Spindexer spindexer) {

        this.hardware = hardware;
        this.callingOpMode = callingOpMode;
        this.spindexer = spindexer;

        hardware.shooterMotorA.setPower(0.0);
        hardware.shooterMotorB.setPower(0.0);
    }

    /// Calculates the expected RPM from the motor's power and voltage
    public static double calculate_rpm(double power, double voltage_v) {
        return ((REGULATOR_POW_TO_RPM_K * power * 100.0) + REGULATOR_POW_TO_RPM_C) * (voltage_v / REGULATOR_BASE_VOLTAGE);
    }

    /// Calculates the power to set for wanted rpm on the motor with the given voltage
    public static double calculate_power_for_rpm(double wanted_rpm, double voltage_v) {
        return ((wanted_rpm * REGULATOR_BASE_VOLTAGE / voltage_v) - REGULATOR_POW_TO_RPM_C ) / REGULATOR_POW_TO_RPM_K / 100.0;
    }

    /// Calculates the radian angle we are shooting at given the servo position
    public static double calculate_rad_angle_for_servo_pos(double servo_pos) {
        return ((ANGLE_SERVO_END - ANGLE_SERVO_START) * servo_pos) + ANGLE_SERVO_START;
    }

    /// Calculates the servo position for a given rad angle
    public static double calculate_servo_pos_for_rad_angle(double angle_rads) {
        return (angle_rads - ANGLE_SERVO_START) / (ANGLE_SERVO_END - ANGLE_SERVO_START);
    }

    /// Calculates the distance factor (times our lowest angle) given our current shooting angle radians.
    ///
    /// See the constants above; this seems to best fit a quadratic.
    public static double calculate_dist_factor_for_angle(double angle_rads) {
        return DISTANCE_ANGLE_FUNC_A * angle_rads * angle_rads + DISTANCE_ANGLE_FUNC_B * angle_rads + DISTANCE_ANGLE_FUNC_C;
    }

    /// Calculates the angle in radians given our distance factor (times our lowest angle).
    ///
    /// Opposite operation of [ShooterPlusPlus::calculate_dist_factor_for_angle].
    ///
    /// See the constants above; this seems to best fit a quadratic.
    public static double calculate_angle_for_distance_factor(double dist_factor) {
        return INVERSE_DIST_ANGLE_FUNC_A * dist_factor * dist_factor + INVERSE_DIST_ANGLE_FUNC_B * dist_factor + INVERSE_DIST_ANGLE_FUNC_C;
    }

    /// Calculates the distance we expect the shooter the hit (at the lowest angle) for the given RPM
    public static double rpm_to_distance_cm(double rpm) {
        // TODO
        return 0.0;
    }

    /// Calculates the RPM to run the shooter at for a given distance (at the lowest angle)
    public static double distance_cm_to_rpm(double distance_cm) {
        // TODO
        return 0.0;
    }

    // Fasttracked calibration
    // dist -> correct angle
    // dist -> rpm to shoot
    public static double distance_cm_to_wanted_angle_rads(double distance_cm) {
        if (distance_cm >= 220.0) {
            return ANGLE_SERVO_START;
        }

        if (distance_cm <= 0.0) {
            return ANGLE_SERVO_END;
        }

        // https://www.wolframalpha.com/input?i=fit+quadratic&assumption=%7B%22F%22%2C+%22QuadraticFitCalculator%22%2C+%22data2%22%7D+-%3E%22%7B%7B0%2C+38.5%7D%2C+%7B41%2C+36.65%7D%2C+%7B78.6%2C+34.8%7D%2C+%7B92.7%2C+32.95%7D%2C+%7B123%2C+31.3%7D%2C+%7B142%2C+29.25%7D%2C+%7B170%2C+27.4%7D%2C+%7B192%2C+23.7%7D%2C+%7B220%2C+20.0%7D%7D%22
        double angle_deg = -0.000245785 * Math.pow(distance_cm, 2) - 0.0279509 * distance_cm + 38.3263;
        double angle_rads = Math.toRadians(angle_deg);

        return Math.max(Math.min(angle_rads, ANGLE_SERVO_END), ANGLE_SERVO_START);
    }

    public static double distance_cm_to_wanted_rpm(double distance_cm) {
        // https://www.wolframalpha.com/input?i=fit+quadratic&assumption=%7B%22F%22%2C+%22QuadraticFitCalculator%22%2C+%22data2%22%7D+-%3E%22%7B%7B0%2C+2900%7D%2C+%7B41%2C+3000%7D%2C+%7B78.6%2C+3000%7D%2C+%7B92.7%2C+3100%7D%2C+%7B123%2C+3200%7D%2C+%7B142%2C+3200%7D%2C+%7B170%2C+3400%7D%2C+%7B192%2C+3500%7D%2C+%7B220%2C+3650%7D%2C+%7B242%2C+3700%7D%2C+%7B265%2C+3800%7D%7D%22
        double rpm = 0.00794905 * Math.pow(distance_cm, 2) + 1.45309 * distance_cm + 2893.77;
        return Math.max(rpm, 0.0);
    }

    public static double parameters_to_distance_cm(double wanted_angle_rads, double wanted_rpm) {
        return 0.0;
    }

    /// Updates the shooter's parameters (RPM, angle) for shooting at a target
    public void update_for_target(TargetInformation target) {
        update_for_distance(target.distance_m);
    }

    /// Updates the shooter's parameters (RPM, angle) for shooting at a specific distance
    public void update_for_distance(double distance_m) {

        wanted_flywheel_rpm = distance_cm_to_wanted_rpm(distance_m * 100.0);
        flywheel_gain = calculate_power_for_rpm(wanted_flywheel_rpm, callingOpMode.hardwareMap.voltageSensor.iterator().next().getVoltage());

        double wanted_angle_rads = Math.min(Math.max(0.0, distance_cm_to_wanted_angle_rads(distance_m * 100.0)), 1.0);
        double wanted_servo_pos = calculate_servo_pos_for_rad_angle(wanted_angle_rads);

        hardware.shooterAngleServo.setPosition(Math.min(Math.max(0.0, wanted_servo_pos), 1.0));
        shooting_distance_m = distance_m;
    }

    /*
    public void update_flywheel_rpm(double flywheel_rpm) {

        if (flywheel_rpm == 0.0) {
            disable_flywheel();
        } else {

            if (!flywheel_enabled) {
                run();
            }

            // Note: won't really work (unless ran in a loop) because we need load voltage here
            set_flywheel_power(calculate_power_for_rpm(flywheel_rpm, callingOpMode.hardwareMap.voltageSensor.iterator().next().getVoltage()));
        }
    }
    */

    ///  Returns how far off we are from the wanted RPM.
    ///
    /// Returs NaN if the shooter is not currently enabled
    public double get_rpm_error() {
        if (!flywheel_enabled) {
            return Double.NaN;
        } else {
            return Math.abs(wanted_flywheel_rpm - last_a_rpm_measurements.average().orElse(0.0));
        }
    }

    public void disable_flywheel() {
        flywheel_enabled = false;

        flywheel_power = 0.0;
        wanted_flywheel_rpm = 0.0;
        set_flywheel_power(0.0);

        started_running_time_ms = 0;
    }

    /// Sets the flywheel to a given power level for the current voltage
    public void set_flywheel_power(double power) {
        hardware.shooterMotorA.setPower(power);
        hardware.shooterMotorB.setPower(power);
        flywheel_power = power;
    }


    /// Returns true when we're okay to fire
    public boolean is_ready_to_fire() {

        boolean flywheel_ready = flywheel_enabled;
        boolean spindexer_not_busy = !spindexer.is_motor_busy() && spindexer.can_move();

        return flywheel_ready && spindexer_not_busy;
    }
    public void run() {
        started_running_time_ms = System.currentTimeMillis();
        flywheel_enabled = true;
    }

    /// Re-measures RPM and runs PID updates
    public void update() {

        long time_ms = System.currentTimeMillis();

        if (!flywheel_enabled) {
            hardware.shooterMotorA.setPower(0.0);
            hardware.shooterMotorB.setPower(0.0);
            return;
        }

        double last_a_pos_ticks = last_a_position_ticks.average().orElse(0.0);
        double last_b_pos_ticks = last_b_position_ticks.average().orElse(0.0);
        double last_time_ms = last_position_time_ms.average().orElse(0.0);

        int a_position_ticks = hardware.shooterMotorA.getCurrentPosition();
        int b_position_ticks = hardware.shooterMotorB.getCurrentPosition();
        double a_delta_position_ticks = a_position_ticks - last_a_pos_ticks;
        double b_delta_position_ticks = b_position_ticks - last_b_pos_ticks;

        double ms_elapsed = time_ms - last_time_ms;

        if (!last_position_time_ms.first().isPresent()) {
            last_position_time_ms.push(time_ms);
            last_a_position_ticks.push(a_position_ticks);
            last_b_position_ticks.push(b_position_ticks);
        }

        boolean started_measurements = last_position_time_ms.first().isPresent();
        boolean enough_elapsed = (time_ms - last_time_ms > 10);
        boolean not_same_reading = (last_a_pos_ticks != a_position_ticks && last_b_pos_ticks != b_position_ticks) || (time_ms - last_time_ms > 5);

        boolean are_measurements_ok = started_measurements && enough_elapsed && not_same_reading;

        if (are_measurements_ok) {

            last_position_time_ms.push(time_ms);

            double a_ticks_per_second = a_delta_position_ticks * 1000.0 / ms_elapsed;
            double a_ticks_per_minute = a_ticks_per_second * 60.0;

            last_a_rpm_measurements.push(a_ticks_per_minute / TICKS_PER_REVOLUTION);
            last_a_position_ticks.push(a_position_ticks);

            double b_ticks_per_second =  b_delta_position_ticks * 1000.0 / ms_elapsed;
            double b_ticks_per_minute = b_ticks_per_second * 60.0;

            last_b_rpm_measurements.push(b_ticks_per_minute / TICKS_PER_REVOLUTION);
            last_b_position_ticks.push(b_position_ticks);
        }

        if (flywheel_power > 0.0 && started_running_time_ms == 0) {
            started_running_time_ms = time_ms;
        }

        if (started_running_time_ms != 0 && Math.abs(flywheel_rel_power) != 1.0) {
            double elapsed_s = (System.currentTimeMillis() - started_running_time_ms) / 1000.0;

            flywheel_rel_power = Math.pow(2.718, elapsed_s / (-STARTUP_TAU));

            if (Math.abs(flywheel_rel_power) < 0.01) {
                flywheel_rel_power = 0.0;
            }
            if (Math.abs(flywheel_rel_power) > 0.95) {
                flywheel_rel_power = Math.signum(flywheel_rel_power);
            }
        }

        flywheel_gain = calculate_power_for_rpm(wanted_flywheel_rpm, callingOpMode.hardwareMap.voltageSensor.iterator().next().getVoltage());
        flywheel_power = flywheel_rel_power * flywheel_gain;

        if (!dry_run) {
            set_flywheel_power(flywheel_power);
        }
    }
}
