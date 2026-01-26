package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "DrivetrainTest", group = "Testing")
public class DrivetrainTest extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.keepHeading = true;

		waitForStart();
		hardware.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
		drivetrain.resetStartingDirection();

		while (opModeIsActive()) {

			Vector2D translation_vector = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
			Vector2D rotation_vector = new Vector2D(gamepad1.right_stick_x, -gamepad1.right_stick_y);

			if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
				translation_vector = new Vector2D(0.0, 0.0);

				if (gamepad1.dpad_up) { translation_vector.y += 0.65; }
				if (gamepad1.dpad_down) { translation_vector.y -= 0.65; }
				if (gamepad1.dpad_right) { translation_vector.x += 0.65; }
				if (gamepad1.dpad_left) { translation_vector.x -= 0.65; }
			}

			if (gamepad1.a) {
				drivetrain.fieldCentricRotation = true;
			}

			if (gamepad1.b) {
				drivetrain.fieldCentricRotation = false;
			}

			if (gamepad1.x) {
				drivetrain.fieldCentricTranslation = true;
			}

			if (gamepad1.y) {
				drivetrain.fieldCentricTranslation = false;
			}

			if (gamepad1.guideWasPressed()) {
				drivetrain.resetStartingDirection();
			}

			drivetrain.update(translation_vector, rotation_vector);

			telemetry.update();
		}
	}
}
