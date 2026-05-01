package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

import java.util.Optional;

public class ShooterPlusPlus {

    /// How many encoder counts mean one revolution on the output axle
    static double TICKS_PER_REVOLUTION = 28.0;

    /// Tau for our startup regulation
    static double STARTUP_TAU = 0.5;

    /// The factor times x in the power to rpm conversion
    static double REGULATOR_POW_TO_RPM_K = 56.58;

    /// The factor times 1 in the power to rpm conversion
    static double REGULATOR_POW_TO_RPM_C = -389.6;

    /// The voltage used for the power to rpm convesion
    static double REGULATOR_BASE_VOLTAGE = 13.43;

    /// The max flywheel power we have
    double flywheel_gain = 1.0;

    /// When we started running the thingo
    long started_running_time_ms = 0;

    OpMode callingOpMode;
    Hardware hardware;
    Spindexer spindexer;

    /// The current flywheel's power - what we're regulating
    public double flywheel_power = 0.0;

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
    public double calculate_rpm(double power, double voltage_v) {
        return ((REGULATOR_POW_TO_RPM_K * power * 100.0) + REGULATOR_POW_TO_RPM_C) * (voltage_v / REGULATOR_BASE_VOLTAGE);
    }

    /// Calculates the power to set for wanted rpm on the motor with the given voltage
    public double calculate_power_for_rpm(double wanted_rpm, double voltage_v) {
        return ((wanted_rpm * REGULATOR_BASE_VOLTAGE / voltage_v) - REGULATOR_POW_TO_RPM_C ) / REGULATOR_POW_TO_RPM_K / 100.0;
    }

    public void disable_flywheel() {
        flywheel_enabled = false;

        flywheel_power = 0.0;
        set_flywheel_power(0.0);

        started_running_time_ms = 0;
    }

    /// Sets the flywheel to a given power level for the nominal voltage
    public void set_flywheel_power(double power) {
        hardware.shooterMotorA.setPower(power);
        hardware.shooterMotorB.setPower(power);
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

        if (started_running_time_ms != 0 && Math.abs(flywheel_power) != flywheel_gain) {

            double elapsed_s = (System.currentTimeMillis() - started_running_time_ms) / 1000.0;

            flywheel_power = Math.pow(2.718, elapsed_s / (-STARTUP_TAU));

            if (Math.abs(flywheel_power) < 0.01) {
                flywheel_power = 0.0;
            }
            if (Math.abs(flywheel_power) > 0.95 * flywheel_gain) {
                flywheel_power = Math.signum(flywheel_power) * flywheel_gain;
            }
        }

        if (!dry_run) {
            set_flywheel_power(flywheel_power);
        }
    }
}
