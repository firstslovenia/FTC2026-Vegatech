package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name = "Auto - blue (near goal)", group = "Examples")
public class BlueAutoNearGoal extends OpMode {

	private Follower follower;
	private Timer pathTimer, actionTimer, opmodeTimer;

	private int pathState;

	private final Pose startPose = new Pose(24.3, 130, Math.toRadians(144));
	private final Pose shootPose = new Pose(56.6, 106, Math.toRadians(144));
	private final Pose endPose = new Pose(56.6, 60, Math.toRadians(180));

	private Path scorePreloaded;
	private PathChain end;

	Hardware hardware;
	Shooter shooter;

	public void buildPaths() {
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
					setPathState(2);
					shooter.update_flywheel_rpm(Shooter.FLYWHEEL_NOMINAL_RPM);
				}
				break;

				// At score position, scoring
			case 2:

				if (pathTimer.getElapsedTime() >= 5000) {
					shooter.update_pusher_power(Shooter.PUSHER_NOMINAL_POWER);

				}

				// After 20s in the opmode, stop scoring
				if (opmodeTimer.getElapsedTime() >= 20000) {
					shooter.update_pusher_power(0.0);
					shooter.update_flywheel_rpm(0.0);

					// Go to the end pos
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

		// Feedback to Driver Hub for debugging
		telemetry.addData("path state", pathState);
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

		shooter = new Shooter(this, hardware);
		shooter.update_pusher_power(0.0);
		shooter.update_flywheel_rpm(0.0);

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
	}

	/**
	 * We do not use this because everything should automatically disable
	 **/
	@Override
	public void stop() {
	}
}
