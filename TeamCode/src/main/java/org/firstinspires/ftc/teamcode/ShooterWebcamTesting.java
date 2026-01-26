package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Shooter Webcam Testing")
public class ShooterWebcamTesting extends LinearOpMode {

	Hardware hardware;
	LedIndicator ledIndicator;
	Webcam webcam;
	ShooterPositioning shooterPositioning = new ShooterPositioning();

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		ledIndicator = new LedIndicator(this, hardware);

		webcam = new Webcam(this, hardware);

		waitForStart();

		while (opModeIsActive()) {
			webcam.update();

			if (webcam.getApriltags().isEmpty()) {
				continue;
			}

			AprilTagDetection detection = webcam.getApriltags().get(0);

			double distance_to_apriltag_m = ShooterPositioning.get_ground_distance_to_apriltag_m(detection);
			TargetInformation target_information = shooterPositioning.compute_target_information(distance_to_apriltag_m, detection.ftcPose.yaw);

			telemetry.addData("Yaw   [deg]", Math.toDegrees(detection.ftcPose.yaw));
			telemetry.addData("Pitch [deg]", Math.toDegrees(detection.ftcPose.pitch));
			telemetry.addData("Roll  [deg]", Math.toDegrees(detection.ftcPose.roll));
			telemetry.addData("Distance (tag)      [m]", distance_to_apriltag_m);
			telemetry.addData("Distance (tag x)    [m]", shooterPositioning.x_distance_to_apriltag_m);
			telemetry.addData("Distance (tag y)    [m]", shooterPositioning.y_distance_to_apriltag_m);
			telemetry.addData("Distance (target)     [m]", target_information.distance_m);
			telemetry.addData("Distance (target x)   [m]", shooterPositioning.x_distance_to_target_m);
			telemetry.addData("Distance (target y)   [m]", shooterPositioning.y_distance_to_target_m);
			telemetry.addData("To turn for zero   [deg]", Math.toDegrees(target_information.angle_distance_to_zero));
			telemetry.addData("Ideal target angle [deg]", Math.toDegrees(target_information.ideal_angle_to_target));
			telemetry.addData("To turn for target [deg]", Math.toDegrees(target_information.angle_distance_rads));
			telemetry.update();
		}
	}
}
