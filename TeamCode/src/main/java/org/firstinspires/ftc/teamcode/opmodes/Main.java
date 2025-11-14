package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Main")
public class Main extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	LedIndicator ledIndicator;
	Shooter shooter;
	Webcam webcam;

	Follower pedroFollower;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.keepHeading = true;

		ledIndicator = new LedIndicator(this, hardware);
		ledIndicator.setPosition(LedIndicator.OFF_POSITION);

		shooter = new Shooter(this, hardware);

		webcam = new Webcam(this, hardware);

		waitForStart();
		hardware.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
		drivetrain.resetStartingDirection();

		pedroFollower = Constants.createFollower(hardwareMap);
		pedroFollower.update();

		while (opModeIsActive()) {

			Pose pos = pedroFollower.getPose();

			telemetry.addData("X pos", pos.getX());
			telemetry.addData("Y pos", pos.getY());
			telemetry.addData("heading (telemetry)", pos.getHeading());

			Vector2D translation_vector = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
			Vector2D rotation_vector = new Vector2D(gamepad1.right_stick_x, -gamepad1.right_stick_y);

			if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
				translation_vector = new Vector2D(0.0, 0.0);

				if (gamepad1.dpad_up) { translation_vector.y += 0.65; }
				if (gamepad1.dpad_down) { translation_vector.y -= 0.65; }
				if (gamepad1.dpad_right) { translation_vector.x += 0.65; }
				if (gamepad1.dpad_left) { translation_vector.x -= 0.65; }
			}

			if (gamepad1.aWasPressed()) {
				drivetrain.fieldCentricRotation = !drivetrain.fieldCentricRotation;
			}

			if (gamepad1.bWasPressed()) {
				drivetrain.fieldCentricTranslation = !drivetrain.fieldCentricTranslation;
			}

			if (gamepad1.x) {
				if (shooter.enabled) {
					shooter.update(0.0);
				} else {
					shooter.update(0.8);
				}
			}

			if (gamepad1.guideWasPressed()) {
				drivetrain.resetStartingDirection();
			}

			// Also works for positioning!
			//
			//follower.updateConstants();
			//follower.updatePose();
			//follower.updateDrivetrain();
			//
			// Just do not use setTeleopDrive

			pedroFollower.update();
			drivetrain.update(translation_vector, rotation_vector);

			if (webcam.last_detections.isEmpty()) {
				ledIndicator.setPosition(LedIndicator.OFF_POSITION);
			} else {
				ledIndicator.setPosition(LedIndicator.GREEN_POSITION);
			}

			webcam.update();

			telemetry.update();
		}
	}
}
