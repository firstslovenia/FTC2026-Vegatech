package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.generic.AngleUtil;
import org.firstinspires.ftc.teamcode.generic.BallColor;

/// Keeps track of which color balls we have and shoots them
///
/// + dir - spin
/// - dir - shoot
public class Spindexer {

    OpMode callingOpMode;
    Hardware hardware;

    /// How many encoder counts mean one revolution on the output axle
    public static int TICKS_PER_REVOLUTION = (int) (28.0 * 27.4);

    /// How long to wait in between shooting balls
    public static final long BALL_SHOOT_DELAY = 700;

    /// How many encoder ticks (here we calculate it from degrees) we allow the PID controller to miss
    public static int PID_TOLERANCE = (TICKS_PER_REVOLUTION * 7) / 360;
    public static double TOLERANCE_RADS = 5.0 * Math.PI / 180.0;

    public static double STARTING_ANGLE = Math.PI / 180.0 * 3.0;

    /// Encoder ticks at 0 degrees, before we run anything
    //public static int STARTING_ENCODER_TICKS = 3963;

    /// The angle to point at to intake into ball 0 - the orange one
    public static double ANGLE_INTAKE_BALL_0 = Math.PI / 6.0;
    public static double ANGLE_INTAKE_BALL_1 = ANGLE_INTAKE_BALL_0 + Math.PI * 2.0 / 3.0;
    public static double ANGLE_INTAKE_BALL_2 = ANGLE_INTAKE_BALL_0 + Math.PI * 4.0 / 3.0;

    /// The angle to point at to shoot ball 0 - the orange one
    public static double ANGLE_SHOOT_BALL_2 = - Math.PI / 180.0 * 103.0;
    public static double ANGLE_SHOOT_BALL_1 = ANGLE_SHOOT_BALL_2 - Math.PI * 2.0 / 3.0;
    public static double ANGLE_SHOOT_BALL_0 = ANGLE_SHOOT_BALL_2 - Math.PI * 4.0 / 3.0;

    public static double ANGLE_HOLD_BALLS_0 = Math.PI / 2.0;

    public static double ANGLE_HOLD_BALLS_1 = Math.PI / 2.0 + Math.PI * 2.0 / 3.0;
    public static double ANGLE_HOLD_BALLS_2 = Math.PI / 2.0 + Math.PI * 4.0 / 3.0;

    /// Balls are numbered ball 0 through ball 2, counterclockwise, starting at more orange to less orange
    public BallColor balls[] = new BallColor[] {BallColor.None, BallColor.None, BallColor.None};

    /// The index of the ball we're actively shooting, if any
    public Integer ball_being_shot = null;
    /// The index of the ball we're prepared to shoot, if any
    public Integer ball_in_shooter = null;
    /// The index of the ball we're rotated to intake, if any
    public Integer ball_to_intake = null;

    /// Set to true when we're spinning the entire spindexer around to see which balls we have
    public boolean in_survey = false;

    /// What time in millis the motor arrived at the correct position
    public Long stopped_being_busy_ms = null;

    /// What time in millis we started having a full spindexer
    ///
    /// Used to remove extra balls from the intake ramp
    public Long started_full_procedure_ms = null;

    /// When our last shot was
    public long last_shot = 0;

    /// Whether or not we were busy in the last loop, used to set stopped_being_busy_ms
    boolean last_was_busy = false;

    public Spindexer(OpMode callingOpMode, Hardware hardware) {
        //hardware.spindexerMotor.setTargetPosition(calculate_nearest_zero_angle_position_for(hardware.spindexerMotor.getCurrentPosition()));

        this.callingOpMode = callingOpMode;
        this.hardware = hardware;
    }

    public void init() {

        DcMotorEx ex = (DcMotorEx) hardware.spindexerMotor;
        ex.setTargetPositionTolerance(PID_TOLERANCE);
        ex.setPositionPIDFCoefficients(8.0);

        hardware.spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.spindexerMotor.setTargetPosition(0);
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
            move_to_intake_for(ball_to_intake);
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
            move_to_shoot_ball(ball_in_shooter);
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

        if (ball_being_shot != null) {
            ball_being_shot = null;
        }

        switch_to_available_intake();

        if (ball_to_intake != null) {
            // We've done our job
            return;
        }

        // There is no available intake, move to the closest holding angle
        double d1 = AngleUtil.ensure_positive_diff(current_angle(), ANGLE_HOLD_BALLS_0);
        double d2 = AngleUtil.ensure_positive_diff(current_angle(), ANGLE_HOLD_BALLS_1);
        double d3 = AngleUtil.ensure_positive_diff(current_angle(), ANGLE_HOLD_BALLS_2);

        if (Math.abs(d1) < Math.abs(d2) && Math.abs(d1) < Math.abs(d3)) {
            move_to_angle_sortwise(ANGLE_HOLD_BALLS_0);
        }
        else if (Math.abs(d2) < Math.abs(d1) && Math.abs(d2) < Math.abs(d3)) {
            move_to_angle_sortwise(ANGLE_HOLD_BALLS_1);
        } else {
            move_to_angle_sortwise(ANGLE_HOLD_BALLS_2);
        }
    }

    /// Goes to shoot balls that we have
    public void switch_to_shooting() {

        if (ball_to_intake != null) {
            ball_to_intake = null;
        }

        if (ball_in_shooter != null) {
            return;
        }

        if (balls[0] != null && balls[0] != BallColor.None) {
            move_to_shoot_ball(0);
        } else if (balls[1] != null && balls[1] != BallColor.None) {
            move_to_shoot_ball(1);
        } else if (balls[2] != null && balls[2] != BallColor.None) {
            move_to_shoot_ball(2);
        } else {
            switch_to_holding_pattern();
        }
    }

    public void update() {

        long now_ms = System.currentTimeMillis();
        boolean is_busy = is_motor_busy();

        if (started_full_procedure_ms != null) {

            if (hardware.intakeMotor.getPower() >= 0.0 && now_ms - started_full_procedure_ms > 200) {
                hardware.intakeMotor.setPower(-0.6);
            }

            if (now_ms - started_full_procedure_ms > 2000) {
                on_spindexer_full_procedure_finished();
            }
        }

        if (ball_being_shot != null) {

            double current_pos = hardware.spindexerMotor.getCurrentPosition();
            double target_pos = hardware.spindexerMotor.getTargetPosition();

            // Shootwise is negative, meaning if current is < target, we overshot it
            boolean has_overshot = current_pos < target_pos;

            if (has_overshot) {
                DcMotorEx ex = (DcMotorEx) hardware.spindexerMotor;
                ex.setTargetPositionTolerance(PID_TOLERANCE);
                ex.setPositionPIDFCoefficients(8.0);
            }

            if (!is_motor_busy()) {
                balls[ball_being_shot] = BallColor.None;

                if (ball_in_shooter != null && balls[ball_in_shooter] != BallColor.None) {
                    long now = System.currentTimeMillis();
                    if (now - last_shot > BALL_SHOOT_DELAY) {
                        shoot_active_ball();
                    }
                } else {
                    switch_to_shooting();
                }
            }
        }

        else if (ball_in_shooter != null && !is_busy) {
            shoot_active_ball();
        }

        if ((ball_in_shooter != null || ball_being_shot != null) && is_empty()) {
            switch_to_holding_pattern();
        }

        // Finish intake, if applicable
        if (ball_to_intake != null && !is_busy) {

            NormalizedRGBA output = hardware.colorSensor.getNormalizedColors();
            double distance_cm = hardware.colorSensor.getDistance(DistanceUnit.CM);

            double koeff_rg = output.red / output.green;
            double rg_off = Math.abs(koeff_rg - 1.0);

            double koeff_gb = output.blue / output.green;
            double gb_off = Math.abs(koeff_gb - 1.0);

            BallColor ball_color = BallColor.None;

            // Do some magic to check if we got something that seems like the right color
            if (rg_off > 0.5 && koeff_gb > 0.5 && output.green > output.red && output.blue > output.red && distance_cm < 7.0) {
                ball_color = BallColor.Green;

            } else if (koeff_gb > 1.25 && rg_off < 0.3 && distance_cm < 7.0) {
                ball_color = BallColor.Purple;
            }

            if (ball_color != BallColor.None) {

                if (in_survey) {
                    mark_survey_ball(ball_color);
                }

                else {
                    balls[ball_to_intake] = ball_color;
                    ball_to_intake = null;
                    switch_to_holding_pattern();

                    if (is_full()) {
                        on_spindexer_full();
                    }
                }
            }
        }

        // Mark the survey ball as empty
        if (in_survey && stopped_being_busy_ms != null && (now_ms - stopped_being_busy_ms >= 200)) {

            // The motor is not busy, and we didn't trigger the intake, so there is no ball there
            if (ball_to_intake != null) {
                mark_survey_ball(BallColor.None);
            } else {
                move_to_intake_for(0);
            }
        }

        if (stopped_being_busy_ms == null && !is_busy && last_was_busy) {
            stopped_being_busy_ms = now_ms;
        }

        if (is_busy) {
            stopped_being_busy_ms = null;
        }

        last_was_busy = is_busy;

        if (ball_being_shot != null) {
            callingOpMode.telemetry.addData("Shooting ball", ball_being_shot);
        }

        if (ball_in_shooter != null) {
            callingOpMode.telemetry.addData("Going to shoot ball", ball_in_shooter);
        }

        if (ball_to_intake != null) {
            callingOpMode.telemetry.addData("Intaking ball", ball_to_intake);
        }

        if (in_survey) {
            callingOpMode.telemetry.addLine("Surveying balls");
        }

        callingOpMode.telemetry.addData("Ball 0", balls[0]);
        callingOpMode.telemetry.addData("Ball 1", balls[1]);
        callingOpMode.telemetry.addData("Ball 2", balls[2]);
    }

    /// Tells the spindexer to start surveying which balls we have
    public void start_survey() {
        in_survey = true;
        ball_in_shooter = null;
        ball_being_shot = null;
        ball_to_intake = null;
        move_to_intake_for(0);
    }

    /// Marks the current ball we're surveying as a specific color and moves on to the next one
    void mark_survey_ball(BallColor color) {
        if (in_survey && ball_to_intake != null) {

            balls[ball_to_intake] = color;

            if (ball_to_intake == 2) {
                in_survey = false;
                ball_to_intake = null;
                switch_to_holding_pattern();
            } else {
                move_to_intake_for(ball_to_intake + 1);
            }
        }
    }

    /// Returns whether the spindexer can physically move without damaging stuff
    public boolean can_move() {
        //return hardware.shooterPusherServo.getPosition() <= 0.3;
        return true;
    }

    /// Tells the spindexer to move to the set angle shootwise
    public void move_to_angle_shootwise(double angle_rads) {

        // Do NOT move!
        if (!can_move()) {
            return;
        }

        if (Math.abs(target_angle() - angle_rads) > TOLERANCE_RADS) {
            int encoder_pos = hardware.spindexerMotor.getCurrentPosition();
            hardware.spindexerMotor.setTargetPosition(nearest_shootwise_pos_at_angle(encoder_pos, angle_rads));
            stopped_being_busy_ms = null;
        }

        hardware.spindexerMotor.setPower(1.0);
        hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /// Tells the spindexer to move to the set angle sortwise
    public void move_to_angle_sortwise(double angle_rads) {

        // Do NOT move!
        if (!can_move()) {
            return;
        }

        if (Math.abs(target_angle() - angle_rads) > TOLERANCE_RADS) {
            int encoder_pos = hardware.spindexerMotor.getCurrentPosition();
            hardware.spindexerMotor.setTargetPosition(nearest_sortwise_pos_at_angle(encoder_pos, angle_rads));
            stopped_being_busy_ms = null;
        }

        hardware.spindexerMotor.setPower(1.0);
        hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void move_to_shoot_ball(int ball) {
        ball_in_shooter = ball;

        switch (ball_in_shooter) {
            case 0:
                move_to_angle_sortwise(ANGLE_SHOOT_BALL_0);
                break;
            case 1:
                move_to_angle_sortwise(ANGLE_SHOOT_BALL_1);
                break;
            case 2:
                move_to_angle_sortwise(ANGLE_SHOOT_BALL_2);
                break;
        }
    }

    public void move_to_shoot_ball_shootwise(int ball) {
        ball_in_shooter = ball;

        switch (ball_in_shooter) {
            case 0:
                move_to_angle_shootwise(ANGLE_SHOOT_BALL_0);
                break;
            case 1:
                move_to_angle_shootwise(ANGLE_SHOOT_BALL_1);
                break;
            case 2:
                move_to_angle_shootwise(ANGLE_SHOOT_BALL_2);
                break;
        }
    }

    public void shoot_active_ball() {
        if (ball_in_shooter == null) {
            return;
        }

        ball_being_shot = ball_in_shooter;
        last_shot = System.currentTimeMillis();

        DcMotorEx ex = (DcMotorEx) hardware.spindexerMotor;
        ex.setTargetPositionTolerance(PID_TOLERANCE * 2);
        ex.setPositionPIDFCoefficients(32.0);

        switch (ball_in_shooter) {
            case 0:
                move_to_shoot_ball_shootwise(2);
                break;
            case 1:
                move_to_shoot_ball_shootwise(0);
                break;
            case 2:
                move_to_shoot_ball_shootwise(1);
                break;
        }
    }

    public void move_to_intake_for(int ball) {
        ball_to_intake = ball;

        switch (ball_to_intake) {
            case 0:
                move_to_angle_sortwise(ANGLE_INTAKE_BALL_0);
                break;
            case 1:
                move_to_angle_sortwise(ANGLE_INTAKE_BALL_1);
                break;
            case 2:
                move_to_angle_sortwise(ANGLE_INTAKE_BALL_2);
                break;
        }
    }

    /// Resets the spindexer to the starting state
    public void reset_state() {
        ball_to_intake = null;
        ball_in_shooter = null;
        ball_being_shot = null;
        in_survey = false;
        move_to_angle_sortwise(Spindexer.STARTING_ANGLE);
    }

    public double current_angle() {
        return calculate_current_angle(hardware.spindexerMotor.getCurrentPosition());
    }

    public double target_angle() {
        return calculate_current_angle(hardware.spindexerMotor.getTargetPosition());
    }

    public boolean is_motor_busy() {
        return hardware.spindexerMotor.isBusy() || Math.abs(hardware.spindexerMotor.getCurrentPosition() - hardware.spindexerMotor.getTargetPosition()) > PID_TOLERANCE;
    }

    /// Returns if we've intaked all balls
    public boolean is_full() {
        return balls[0] != BallColor.None && balls[1] != BallColor.None && balls[2] != BallColor.None;
    }

    /// Returns if we've are empty
    public boolean is_empty() {
        return balls[0] == BallColor.None && balls[1] == BallColor.None && balls[2] == BallColor.None;
    }

    /// Runs when the spindexer becomes 100% full (we've intaked three balls)
    public void on_spindexer_full() {
        started_full_procedure_ms = System.currentTimeMillis();
        callingOpMode.gamepad1.rumble(200);
    }

    public void on_spindexer_full_procedure_finished() {
        hardware.intakeMotor.setPower(0.0);
        started_full_procedure_ms = null;
    }

    /// Returns how far (via ticks) we are into the current loop
    public static int calculate_relative_loop_ticks(int encoder_position) {
        //int moved_since_zero = encoder_position - STARTING_ENCODER_TICKS;
        int moved_since_zero = encoder_position;
        return Math.floorMod(moved_since_zero, TICKS_PER_REVOLUTION);
    }

    /// Computes the nearest position to the given position which puts the spidexer into the wanted angle, going shootwise
    public static int nearest_shootwise_pos_at_angle(int encoder_position, double angle_rads) {

        double current_angle_rads = calculate_current_angle(encoder_position);
        double angle_diff = AngleUtil.ensure_negative_diff(current_angle_rads, angle_rads);

        int angle_diff_ticks = (int) ((angle_diff / Math.PI / 2.0) * TICKS_PER_REVOLUTION);
        return encoder_position + angle_diff_ticks;
    }

    /// Computes the nearest position to the given position which puts the spidexer into the wanted angle, going sortwise
    public static int nearest_sortwise_pos_at_angle(int encoder_position, double angle_rads) {

        double current_angle_rads = calculate_current_angle(encoder_position);
        double angle_diff = AngleUtil.ensure_positive_diff(current_angle_rads, angle_rads);

        int angle_diff_ticks = (int) ((angle_diff / Math.PI / 2.0) * TICKS_PER_REVOLUTION);
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