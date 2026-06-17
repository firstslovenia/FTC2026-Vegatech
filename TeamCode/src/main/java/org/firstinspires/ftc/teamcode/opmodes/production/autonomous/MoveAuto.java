package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;

@Autonomous(name = "NO TOUCHIE!!!", group = "Examples")
public class MoveAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Must be set by derived classes!!!!
    public Pose startPose;
    public Pose lookAtObeliskPose;
    public Pose shootPose;
    public Pose endPose;
    public Pose pickupPose;
    public Pose goalPose;

    public ShooterPositioning positioning = new ShooterPositioning();
    public TargetInformation targetInformation = null;

    public static double ballPickupXOffsetRed = 30.0;
    public static double ballPickupXOffsetBlue = -30.0;

    /// X offset to apply to final position to pickup balls
    public double ballPickupXOffset = ballPickupXOffsetRed;

    public static Pose blueGoalPose = new Pose(8.0, 135.0);
    public static Pose redGoalPose = new Pose(132.0, 136.0);

    // Target poses for far autonomous modes
    public static Pose blueGoalFarPose = new Pose(15.0, 136.0);
    public static Pose redGoalFarPose = new Pose(129.0, 136.0);

    public static Pose obeliskPose = new Pose(72.0, 144.0);

    //private final Pose startPose = new Pose(119.6, 130, Math.toRadians(36));
    //private final Pose shootPose = new Pose(87.5, 108, Math.toRadians(36));
    //private final Pose endPose = new Pose(87.5, 60, Math.toRadians(0));

    private Path move_to_look_at_obelisk;
    private Path move_to_shoot;

    private Path rotate_to_correct_heading;

    private Path move_to_pickup;
    private PathChain move;

    Hardware hardware;
    ShooterPlusPlus shooter;
    Spindexer spindexer;
    Webcam webcam;

    /// How many balls we've already scored
    int balls_scored = 0;

    /// When our last shot was
    long last_shot_time_ms = 0;

    /// When we started staring at the obelisk
    long started_looking_at_obelisk_ms = 0;

    /// The distance between our shooting pos and the goal
    double shooting_distance_m = 0.0;

    public void buildPaths() {
        move = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(4);
                break;

            case 4:
                spindexer.switch_to_holding_pattern();
                spindexer.move_to_angle_sortwise(0.0);

                follower.followPath(move, true);
                setPathState(5);
                break;

            // Going to the end position
            case 5:
                if (!follower.isBusy() || opmodeTimer.getElapsedTime() >= 27000) {
                    setPathState(7);
                }
                break;

            // Finished
            case 7:

                follower.breakFollowing();

                hardware.intakeMotor.setPower(0.0);

                spindexer.ball_to_intake = null;
                spindexer.ball_in_shooter = null;
                spindexer.ball_being_shot = null;
                spindexer.in_survey = false;
                spindexer.move_to_angle_sortwise(0.0);
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        shooter.update();
        spindexer.update();

        // Feedback to Driver Hub for debugging
        telemetry.addData("shoot dist [cm]", shooting_distance_m);
        telemetry.addData("shooter is ready", shooter.is_ready_to_fire());
        telemetry.addData("shooter RPM delta", shooter.get_rpm_error());
        telemetry.addData("path state", pathState);
        telemetry.addData("balls shot", balls_scored);
        telemetry.addData("spx can move", spindexer.can_move());
        telemetry.addData("spx busy", spindexer.is_motor_busy());
        telemetry.addData("spx Position", hardware.spindexerMotor.getCurrentPosition());
        telemetry.addData("spx Wanted pos", hardware.spindexerMotor.getTargetPosition());
        telemetry.addData("spx Nearest 0 angle", spindexer.nearest_shootwise_pos_at_angle(hardware.spindexerMotor.getCurrentPosition(), 0.0));
        telemetry.addData("spx Angle", Math.toDegrees(spindexer.current_angle()));
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("order: ", webcam.order);
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        hardware = new Hardware(this);
        hardware.init();

        spindexer = new Spindexer(this, hardware);
        spindexer.init();

        shooter = new ShooterPlusPlus(this, hardware, spindexer);

        webcam = new Webcam(this, hardware);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);

        spindexer.start_survey();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}