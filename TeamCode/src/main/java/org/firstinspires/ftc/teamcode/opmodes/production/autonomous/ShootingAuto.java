package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ColorOrder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.BallColor;
import org.firstinspires.ftc.teamcode.opmodes.production.teleop.Main;

@Autonomous(name = "NO TOUCHIE!!", group = "Examples")
public class ShootingAuto extends OpMode {

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
    private PathChain end;

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

        targetInformation = positioning.compute_target_information_for_two_pos(ShooterPositioning.to_pose2d(shootPose), ShooterPositioning.to_pose2d(goalPose));
        shootPose = shootPose.setHeading(targetInformation.ideal_angle_to_target);

        // Pedropathing doesn't work if it has no work to do
        lookAtObeliskPose = shootPose.withY(shootPose.getY() + 10.0);

        double obelisk_delta_y = obeliskPose.getY() - lookAtObeliskPose.getY();
        double obelisk_delta_x = obeliskPose.getX() - lookAtObeliskPose.getX();
        double obelisk_angle = Drivetrain.getMagnitudeAndPhiFor(obelisk_delta_x, obelisk_delta_y).second;

        lookAtObeliskPose = lookAtObeliskPose.setHeading(obelisk_angle);

        // Calculate pickup pose
        pickupPose = endPose.withX(endPose.getX() + ballPickupXOffset);

        // -- actually build paths
        move_to_look_at_obelisk = new Path(new BezierLine(startPose, lookAtObeliskPose));
        move_to_look_at_obelisk.setLinearHeadingInterpolation(startPose.getHeading(), lookAtObeliskPose.getHeading());

        move_to_shoot = new Path(new BezierLine(lookAtObeliskPose, shootPose));
        move_to_shoot.setConstantHeadingInterpolation(shootPose.getHeading());

        //move_to_pickup = new Path(new BezierLine(endPose, pickupPose));
        //move_to_pickup.setConstantHeadingInterpolation(pickupPose.getHeading());

        end = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(move_to_look_at_obelisk, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            // Start going to the shoot position
            case 2:
                long now_ms = System.currentTimeMillis();

                if (started_looking_at_obelisk_ms == 0) {
                    started_looking_at_obelisk_ms = now_ms;
                }

                webcam.update();

                if (webcam.order != ColorOrder.Unknown || now_ms - started_looking_at_obelisk_ms >= 7_000 || opmodeTimer.getElapsedTime() >= 13_000) {
                    follower.followPath(move_to_shoot, true);
                    setPathState(3);
                }
                break;

            // Go to the shooting position
            case 3:
                if (!follower.isBusy()) {
                    shooter.update_for_target(targetInformation);
                    shooter.run();
                    setPathState(4);
                }
                break;

            // At score position, scoring
            case 4:

                long now = System.currentTimeMillis();

                if (spindexer.in_survey || !spindexer.can_move()) {
                    return;
                }

                // Pretend we know the pattern
                if (now - last_shot_time_ms > 1000) {
                   if (spindexer.ball_in_shooter != null) {
                        spindexer.shoot_active_ball();
                        last_shot_time_ms = now;
                    } else {
                        spindexer.switch_to_shooting();
                    }
                }

                // After 22s in the opmode, stop trying to score
                if (opmodeTimer.getElapsedTime() >= 22000) {
                    shooter.disable_flywheel();

                    spindexer.switch_to_holding_pattern();
                    spindexer.move_to_angle_sortwise(0.0);

                    follower.followPath(end, true);
                    setPathState(5);
                }
                break;

            // Going to the end position
            case 5:
                if (!follower.isBusy() || opmodeTimer.getElapsedTime() >= 29000) {

                    /*if (opmodeTimer.getElapsedTime() >= 26000) {
                        setPathState(7);
                    }*/

                    setPathState(7);

                    // Try in to intake before
                    /*spindexer.switch_to_holding_pattern();

                    if (spindexer.ball_to_intake == null) {
                        setPathState(7);
                    }

                    hardware.intakeMotor.setPower(1.0);
                    follower.followPath(move_to_pickup);
                    setPathState(6);*/
                }
                break;

            // Try to pickup balls
            case 6:

                if (opmodeTimer.getElapsedTime() >= 27000) {
                    setPathState(7);
                }

                break;

            // Finished
            case 7:

                follower.breakFollowing();

                hardware.intakeMotor.setPower(0.0);
                spindexer.reset_state();
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
        spindexer.balls[0] = BallColor.Green;
        spindexer.balls[1] = BallColor.Green;
        spindexer.balls[2] = BallColor.Green;

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

        hardware.odometry.setPosition(ShooterPositioning.to_pose2d(startPose));

        hardware.cameraAngleServo.setPosition(Main.LOWER_CAMERA_POSITION);

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