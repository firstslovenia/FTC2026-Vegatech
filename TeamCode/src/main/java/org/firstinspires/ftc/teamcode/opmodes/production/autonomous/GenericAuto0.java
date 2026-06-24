package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.opmodes.production.teleop.Main;

@Autonomous(name = "NO TOUCHIE!", group = "Examples")
public class GenericAuto0 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Must be set by derived classes!!!!
    public Pose startPose;
    public Pose lookAtObeliskPose;
    public Pose intermediatePose;
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
    private PathChain move_2;

    Hardware hardware;
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
                .addPath(new BezierLine(startPose, intermediatePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        move_2 = follower.pathBuilder()
                .addPath(new BezierLine(intermediatePose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(4);
                break;

            case 4:
                follower.followPath(move, true);
                setPathState(5);
                break;

            // Going to the intermediate position
            case 5:
                if (!follower.isBusy() || opmodeTimer.getElapsedTime() >= 20000) {
                    follower.followPath(move_2, true);
                    setPathState(6);
                }
                break;

            // Going to the end position
            case 6:
                if (!follower.isBusy() || opmodeTimer.getElapsedTime() >= 29000) {
                    setPathState(7);
                }
                break;

            // Finished
            case 7:

                follower.breakFollowing();
                hardware.intakeMotor.setPower(0.0);
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

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("x (gobilda, m)", hardware.odometry.getPosition().getX(DistanceUnit.METER));
        telemetry.addData("y (goblida, m)", hardware.odometry.getPosition().getY(DistanceUnit.METER));
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

        hardware.odometry.setPosition(ShooterPositioning.to_pose2d(startPose));

        hardware.cameraAngleServo.setPosition(Main.LOWER_CAMERA_POSITION);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}