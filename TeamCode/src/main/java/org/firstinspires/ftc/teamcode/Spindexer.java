package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.generic.BallColor;
import org.firstinspires.ftc.teamcode.generic.GenericPIDController;

/// Keeps track of which color balls we have and
public class Spindexer {

    OpMode callingOpMode;
    Hardware hardware;

    /// How many encoder counts mean one revolution on the output axle
    public static int TICKS_PER_REVOLUTION = (int) (28.0 * 27.4);

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

    /// Balls are numbered ball 0 through ball 2, counterclockwise, starting at more orange to less orange
    public BallColor balls[] = new BallColor[] {BallColor.None, BallColor.None, BallColor.None};

    /// The index of the ball we're rotated to shoot, if any
    public Integer ball_in_shooter = null;
    /// The index of the ball we're rotated to intake, if any
    public Integer ball_to_intake = null;

    public Spindexer(OpMode callingOpMode, Hardware hardware) {
        //hardware.spindexerMotor.setTargetPosition(calculate_nearest_zero_angle_position_for(hardware.spindexerMotor.getCurrentPosition()));

        this.callingOpMode = callingOpMode;
        this.hardware = hardware;
    }

    public void init() {
        hardware.spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /// Tells the spindexer to move to the set angle
    public void move_to_angle(double angle_rads) {

        if (Math.abs(target_angle() - angle_rads) > Math.PI / 180.0) {
            int encoder_pos = hardware.spindexerMotor.getCurrentPosition();
            hardware.spindexerMotor.setTargetPosition(calculate_nearest_position_at_angle(encoder_pos, angle_rads));
        }

        hardware.spindexerMotor.setPower(0.5);

        if (hardware.spindexerMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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
        int moved_around_loop = calculate_relative_loop_ticks(encoder_position);

        // Make a positive angle between 0 and 2 PI
        if (angle_rads < 0.0) {
            angle_rads += Math.PI * 2.0;
        }

        while (angle_rads > Math.PI * 2.0) {
            angle_rads -= Math.PI * 2.0;
        }

        int angle_in_ticks = (int) ((angle_rads / Math.PI / 2.0) * TICKS_PER_REVOLUTION);
        int negative_angle_in_ticks = (int) (((angle_rads - Math.PI * 2.0) / Math.PI / 2.0) * TICKS_PER_REVOLUTION);

        int distance_to_this_loop = -moved_around_loop + angle_in_ticks;
        int distance_to_previous_loop = -moved_around_loop + negative_angle_in_ticks;

        if (Math.abs(distance_to_previous_loop) < distance_to_this_loop) {
            return encoder_position + distance_to_previous_loop;
        } else {
            return encoder_position + distance_to_this_loop;
        }
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
