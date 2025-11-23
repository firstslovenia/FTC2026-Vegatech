package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
		drivetrain.resetStartingDirection();

		pedroFollower = Constants.createFollower(hardwareMap);
		pedroFollower.update();

		while (opModeIsActive()) {

			Pose pos = pedroFollower.getPose();

			telemetry.addData("Shooter power", hardware.shooterMotor.getPower());
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

			//if (gamepad1.aWasPressed()) {
			//	drivetrain.keepHeading = !drivetrain.keepHeading;
			//}

			//if (gamepad1.bWasPressed()) {
			//	drivetrain.fieldCentricTranslation = !drivetrain.fieldCentricTranslation;
			//}

			// Reset robot to 90 degrees
			if (gamepad1.a) {
				drivetrain.wanted_heading = Math.PI / 2.0;
			}

			if (gamepad1.xWasPressed()) {
				if (shooter.flywheel_enabled) {
					shooter.update_flywheel_rpm(0.0);
				} else {
					shooter.update_flywheel_rpm(Shooter.FLYWHEEL_NOMINAL_RPM);
				}
			}

			if (gamepad1.right_trigger > 0.1) {
				shooter.update_pusher_power(Shooter.PUSHER_NOMINAL_POWER);
			} else if (gamepad1.left_trigger > 0.1) {
				shooter.update_pusher_power(-Shooter.PUSHER_NOMINAL_POWER);
			} else {
				shooter.update_pusher_power(0.0);
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
