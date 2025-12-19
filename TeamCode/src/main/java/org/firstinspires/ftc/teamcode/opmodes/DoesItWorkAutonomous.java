package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name = "\"Does It Work\" Autonomous")
public class DoesItWorkAutonomous extends OpMode {
	public Follower follower; // Pedro Pathing follower instance
	private int pathState; // Current autonomous path state (state machine)
	private Paths paths; // Paths defined in the Paths class

	@Override
	public void init() {
		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(0.0, 0.0, Math.toRadians(180)));

		paths = new Paths(follower); // Build paths
	}

	@Override
	public void loop() {
		follower.update(); // Update Pedro Pathing
		autonomousPathUpdate(); // Update autonomous state machine

		telemetry.addData("Path State", pathState);
		telemetry.addData("X", follower.getPose().getX());
		telemetry.addData("Y", follower.getPose().getY());
		telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
		telemetry.update();
	}

	public static class Paths {

		public PathChain Path1;

		public Paths(Follower follower) {
			Path1 = follower
				.pathBuilder()
				.addPath(
					new BezierLine(new Pose(0.000, 0.000), new Pose(0.000, 48.000))
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
				.build();
		}
	}

	public int autonomousPathUpdate() {

		follower.followPath(paths.Path1, true);

		// Add your state machine Here
		// Access paths with paths.pathName
		// Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
		return pathState;
	}
}
