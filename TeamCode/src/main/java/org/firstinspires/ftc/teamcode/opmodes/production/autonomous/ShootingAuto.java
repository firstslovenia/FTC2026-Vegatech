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
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.BallColor;

@Autonomous(name = "NO TOUCHIE!!", group = "Examples")
public class ShootingAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Must be set by derived classes!!!!
    public Pose startPose;
    public Pose shootPose;
    public Pose endPose;

    public Pose goalPose;

    // The plus here adds the range to the actual board of the goal
    // instead of its center.
    //
    // We add that because the pose we're calculating with is the center of the robot
    // and not the shooter front.
    public Pose blueGoalPose = new Pose(12.0 + 4, 137.0 - 6);
    public Pose redGoalPose = new Pose(132.0 - 4, 137.0 - 6);

    //private final Pose startPose = new Pose(119.6, 130, Math.toRadians(36));
    //private final Pose shootPose = new Pose(87.5, 108, Math.toRadians(36));
    //private final Pose endPose = new Pose(87.5, 60, Math.toRadians(0));

    private Path scorePreloaded;
    private PathChain end;

    Hardware hardware;
    Shooter shooter;
    Spindexer spindexer;
    Webcam webcam;

    /// How many balls we've already scored
    int balls_scored = 0;

    /// When our last shot was
    long last_shot_time_ms = 0;

    /// The distance between our shooting pos and the goal
    double shooting_distance_m = 0.0;

    public void buildPaths() {

        double shooting_delta_y = goalPose.getY() - shootPose.getY();
        double shooting_delta_x = goalPose.getX() - shootPose.getX();

        // LOL!
        double angle = Drivetrain.getMagnitudeAndPhiFor(shooting_delta_x, shooting_delta_y).second;

        shootPose = shootPose.setHeading(angle);

        double shooting_distance_in = Math.sqrt(shooting_delta_x * shooting_delta_x + shooting_delta_y + shooting_delta_y);
        shooting_distance_m = shooting_distance_in / 39.37;

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreloaded = new Path(new BezierLine(startPose, shootPose));
        scorePreloaded.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        end = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // 0 - start going to score
            case 0:
                follower.followPath(scorePreloaded, true);
                setPathState(1);
                break;

            // Going to score position
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    shooter.update_rpm_for_distance_m(shooting_distance_m);

                    setPathState(2);
                }
                break;

            // At score position, scoring
            case 2:

                long now = System.currentTimeMillis();

                if (spindexer.in_survey || !spindexer.can_move()) {
                    return;
                }

                // Pretend we know the pattern
                BallColor next = webcam.order.color_for_ith(balls_scored);

                if (spindexer.ball_in_shooter == null) {
                    spindexer.switch_to_coloured_ball(next);

                    // There is none
                    if (spindexer.ball_in_shooter == null) {
                        spindexer.switch_to_coloured_ball(next.other());
                    }
                }

                if (shooter.is_ready_to_fire() && spindexer.ball_in_shooter != null && now - last_shot_time_ms > 500) {
                   shooter.fire();

                   spindexer.balls[spindexer.ball_in_shooter] = BallColor.None;
                   spindexer.ball_in_shooter = null;

                   balls_scored += 1;
                   last_shot_time_ms = System.currentTimeMillis();
                }

                // After 20s in the opmode, stop scoring
                if ((balls_scored >= 3 && now - last_shot_time_ms > 1000) || opmodeTimer.getElapsedTime() >= 20000) {
                    shooter.update_flywheel_rpm(0.0);
                    spindexer.switch_to_holding_pattern();

                    follower.followPath(end, true);
                    setPathState(3);
                }
                break;

            // Going to the end position
            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            // Finished
            case 4:
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
        telemetry.addData("path state", pathState);
        telemetry.addData("balls shot", balls_scored);
        telemetry.addData("spx: can move", spindexer.can_move());
        telemetry.addData("spx: busy", spindexer.is_motor_busy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
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

        shooter = new Shooter(this, hardware, spindexer);
        shooter.reset_shooter_pusher();

        webcam = new Webcam(this, hardware);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
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
        opmodeTimer.resetTimer();
        setPathState(0);

        spindexer.init();
        spindexer.start_survey();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}