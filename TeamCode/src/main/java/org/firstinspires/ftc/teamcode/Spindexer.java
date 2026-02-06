package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.generic.AngleUtil;
import org.firstinspires.ftc.teamcode.generic.BallColor;

/// Keeps track of which color balls we have and
public class Spindexer {

    OpMode callingOpMode;
    Hardware hardware;

    /// How many encoder counts mean one revolution on the output axle
    public static int TICKS_PER_REVOLUTION = (int) (28.0 * 27.4);

    /// How many encoder ticks (here we calculate it from degrees) we allow the PID controller to miss
    public static int PID_TOLERANCE = (TICKS_PER_REVOLUTION * 5) / 360;

    /// Encoder ticks at 0 degrees, before we run anything
    //public static int STARTING_ENCODER_TICKS = 3963;

    /// The angle to point at to intake into ball 0 - the orange one
    public static double ANGLE_INTAKE_BALL_0 = Math.PI / 6.0;
    public static double ANGLE_INTAKE_BALL_1 = ANGLE_INTAKE_BALL_0 - Math.PI * 2.0 / 3.0;
    public static double ANGLE_INTAKE_BALL_2 = ANGLE_INTAKE_BALL_0 - Math.PI * 4.0 / 3.0;

    /// The angle to point at to shoot ball 0 - the orange one
    public static double ANGLE_SHOOT_BALL_0 = - Math.PI / 3.0;
    public static double ANGLE_SHOOT_BALL_1 = ANGLE_SHOOT_BALL_0 - Math.PI * 2.0 / 3.0;
    public static double ANGLE_SHOOT_BALL_2 = ANGLE_SHOOT_BALL_0 - Math.PI * 4.0 / 3.0;

    public static double ANGLE_HOLD_BALLS_0 = Math.PI / 2.0;

    public static double ANGLE_HOLD_BALLS_1 = Math.PI / 2.0 + Math.PI * 2.0 / 3.0;
    public static double ANGLE_HOLD_BALLS_2 = Math.PI / 2.0 + Math.PI * 4.0 / 3.0;

    /// Balls are numbered ball 0 through ball 2, counterclockwise, starting at more orange to less orange
    public BallColor balls[] = new BallColor[] {BallColor.None, BallColor.None, BallColor.None};

    /// The index of the ball we're rotated to shoot, if any
    public Integer ball_in_shooter = null;
    /// The index of the ball we're rotated to intake, if any
    public Integer ball_to_intake = null;

    public Long started_being_busy_ms = null;

    public Spindexer(OpMode callingOpMode, Hardware hardware) {
        //hardware.spindexerMotor.setTargetPosition(calculate_nearest_zero_angle_position_for(hardware.spindexerMotor.getCurrentPosition()));

        this.callingOpMode = callingOpMode;
        this.hardware = hardware;
    }

    public void init() {
        hardware.spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) hardware.spindexerMotor).setTargetPositionTolerance(PID_TOLERANCE);
    }

    /// Moves to the next available ball spot we can intake into.
    ///
    /// If none is available, ball_to_intake will still be null.
    public void switch_to_available_intake() {

        // Note: it isn't really worth optimizing this
        if (ball_in_shooter != null) {
            ball_in_shooter = null;
        }

        for (int i = 0; i < 3; i++) {
            if (balls[i] == BallColor.None) {
                ball_to_intake = i;
                break;
            }
        }

        if (ball_to_intake != null) {
            switch (ball_to_intake) {
                case 0:
                    move_to_angle(ANGLE_INTAKE_BALL_0);
                    break;
                case 1:
                    move_to_angle(ANGLE_INTAKE_BALL_1);
                    break;
                case 2:
                    move_to_angle(ANGLE_INTAKE_BALL_2);
                    break;
            }
        }
    }

    /// Moves to the next available green ball to shoot.
    ///
    /// If none is available, ball_in_shooter will still be null.
    public void switch_to_coloured_ball(BallColor color) {

        // Note: it isn't really worth optimizing this
        if (ball_to_intake != null) {
            ball_to_intake = null;
        }

        for (int i = 0; i < 3; i++) {
            if (balls[i] == color) {
                ball_in_shooter = i;
                break;
            }
        }

        if (ball_in_shooter != null) {
            switch (ball_in_shooter) {
                case 0:
                    move_to_angle(ANGLE_SHOOT_BALL_0);
                    break;
                case 1:
                    move_to_angle(ANGLE_SHOOT_BALL_1);
                    break;
                case 2:
                    move_to_angle(ANGLE_SHOOT_BALL_2);
                    break;
            }
        }
    }

    /// Moves to the passive holding angle.
    ///
    /// If available, goes to the next intake position.
    ///
    /// If not, goes to the nearest holding pattern for keeping all the balls inside
    public void switch_to_holding_pattern() {

        if (ball_to_intake != null) {
            // We're already intaking a ball!
            return;
        }

        if (ball_in_shooter != null) {
            ball_in_shooter = null;
        }

        switch_to_available_intake();

        if (ball_to_intake != null) {
            // We've done our job; don't go into intake mode, just keep the spindexer there
            ball_to_intake = null;
            return;
        }

        double d1 = AngleUtil.calculate_best_angle_diff_for(current_angle(), ANGLE_HOLD_BALLS_0);
        double d2 = AngleUtil.calculate_best_angle_diff_for(current_angle(), ANGLE_HOLD_BALLS_1);
        double d3 = AngleUtil.calculate_best_angle_diff_for(current_angle(), ANGLE_HOLD_BALLS_2);

        if (Math.abs(d1) < Math.abs(d2) && Math.abs(d1) < Math.abs(d3)) {
            move_to_angle(ANGLE_HOLD_BALLS_0);
        }
        else if (Math.abs(d2) < Math.abs(d1) && Math.abs(d2) < Math.abs(d3)) {
            move_to_angle(ANGLE_HOLD_BALLS_1);
        } else {
            move_to_angle(ANGLE_HOLD_BALLS_2);
        }
    }

    public void update() {

        long now_ms = System.currentTimeMillis();

        if (started_being_busy_ms != null && now_ms - started_being_busy_ms > 100) {
            if (!hardware.spindexerMotor.isBusy()) {
                started_being_busy_ms = null;
            }
        }

        // Finish intake, if applicable
        if (ball_to_intake != null && !hardware.spindexerMotor.isBusy()) {

            NormalizedRGBA output = hardware.colorSensor.getNormalizedColors();
            double distance_cm = hardware.colorSensor.getDistance(DistanceUnit.CM);

            double koeff_rg = output.red / output.green;
            double rg_off = Math.abs(koeff_rg - 1.0);

            double koeff_gb = output.blue / output.green;
            double gb_off = Math.abs(koeff_gb - 1.0);

            // Do some magic to check if we got something that seems like the right color
            if (rg_off > 0.5 && koeff_gb > 0.5 && output.green > output.red && output.blue > output.red && distance_cm < 7.0) {
                balls[ball_to_intake] = BallColor.Green;
                ball_to_intake = null;
                switch_to_holding_pattern();

            } else if (koeff_gb > 1.25 && rg_off < 0.3 && distance_cm < 7.0) {
                balls[ball_to_intake] = BallColor.Purple;
                ball_to_intake = null;
                switch_to_holding_pattern();
            }
        }

        callingOpMode.telemetry.addData("Spindexer b0", balls[0]);
        callingOpMode.telemetry.addData("Spindexer b1", balls[1]);
        callingOpMode.telemetry.addData("Spindexer b2", balls[2]);

        if (ball_in_shooter != null) {
            callingOpMode.telemetry.addData("Shooting ball", ball_in_shooter);
        }

        if (ball_to_intake != null) {
            callingOpMode.telemetry.addData("Intaking ball", ball_to_intake);
        }
    }

    /// Tells the spindexer to move to the set angle
    public void move_to_angle(double angle_rads) {

        if (Math.abs(target_angle() - angle_rads) > Math.PI / 180.0) {
            int encoder_pos = hardware.spindexerMotor.getCurrentPosition();
            hardware.spindexerMotor.setTargetPosition(calculate_nearest_position_at_angle(encoder_pos, angle_rads));

            started_being_busy_ms = System.currentTimeMillis();
        }

        hardware.spindexerMotor.setPower(1.0);
        hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double current_angle() {
        return calculate_current_angle(hardware.spindexerMotor.getCurrentPosition());
    }

    public double target_angle() {
        return calculate_current_angle(hardware.spindexerMotor.getTargetPosition());
    }

    /// Returns how far (via ticks) we are into the current loop
    public static int calculate_relative_loop_ticks(int encoder_position) {
        //int moved_since_zero = encoder_position - STARTING_ENCODER_TICKS;
        int moved_since_zero = encoder_position;
        return Math.floorMod(moved_since_zero, TICKS_PER_REVOLUTION);
    }

    /// Computes the nearest position to the given position which puts the spidexer into the wanted angle
    public static int calculate_nearest_position_at_angle(int encoder_position, double angle_rads) {

        double current_angle_rads = calculate_current_angle(encoder_position);
        double best_angle_diff = AngleUtil.calculate_best_angle_diff_for(current_angle_rads, angle_rads);

        int angle_diff_ticks = (int) ((best_angle_diff / Math.PI / 2.0) * TICKS_PER_REVOLUTION);
        return encoder_position + angle_diff_ticks;
    }

    /// Calculate the current angle of the spindexer
    ///
    /// Angle 0 means that grey parts and orange parts on the spindexer
    /// and walls are lined up, and the connecting line on the spindexer is
    /// dead center in the shooter pusher servo hole
    ///
    /// Always returns positive angles
    public static double calculate_current_angle(int encoder_pos) {
        return (double) calculate_relative_loop_ticks(encoder_pos) / TICKS_PER_REVOLUTION * Math.PI * 2.0;
    }
}
