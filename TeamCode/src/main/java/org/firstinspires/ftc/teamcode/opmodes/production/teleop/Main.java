package org.firstinspires.ftc.teamcode.opmodes.production.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Main", group = "Production")
public class Main extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	LedIndicator ledIndicator;
	Shooter shooter;
    Webcam webcam;
	Follower pedroFollower;

    Team team = null;

    /// The TargetInformation of the target we're currently rotating towards
    TargetInformation target_to_rotate_to = null;
    /// The wanted heading of the target for target_to_rotate_to
    double wanted_heading_for_target = Double.NaN;
    long last_updated_camera_ms = 0;
    static long CAMERA_UPDATE_DELAY_MS = 50;

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

            long now = System.currentTimeMillis();

            // If shooter is on, update LED
            if (shooter.flywheel_enabled) {

                double rpm_error = shooter.get_rpm_error();

                if (Double.isNaN(shooter.shooting_distance_m)) {
                    if (rpm_error > 100.0) {
                        ledIndicator.setPosition(LedIndicator.ORANGE_POSITION);
                    } else if (rpm_error > 20.0) {
                        ledIndicator.setPosition(LedIndicator.INDIGO_POSITION);
                    } else {
                        ledIndicator.setPosition(LedIndicator.BLUE_POSITION);
                    }
                } else {
                    if (rpm_error > 100.0) {
                        ledIndicator.setPosition(LedIndicator.RED_POSITION);
                    } else if (rpm_error > 20.0) {
                        ledIndicator.setPosition(LedIndicator.YELLOW_POSITION);
                    } else {
                        ledIndicator.setPosition(LedIndicator.GREEN_POSITION);
                    }

                    telemetry.addData("Shooter distance (cm)", shooter.shooting_distance_m * 100.0);
                }

                telemetry.addData("Shooter RPM", shooter.wanted_flywheel_rpm);
                telemetry.addData("Shooter Error", rpm_error);
            }

			Vector2D translation_vector = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
			Vector2D rotation_vector = new Vector2D(gamepad1.right_stick_x, -gamepad1.right_stick_y);

			if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
				translation_vector = new Vector2D(0.0, 0.0);

				if (gamepad1.dpad_up) { translation_vector.y += 0.65; }
				if (gamepad1.dpad_down) { translation_vector.y -= 0.65; }
				if (gamepad1.dpad_right) { translation_vector.x += 0.65; }
				if (gamepad1.dpad_left) { translation_vector.x -= 0.65; }
			}

			// Reset robot to 90 degrees
			if (gamepad1.a) {
				drivetrain.wanted_heading = Math.PI / 2.0;
			}

            // Rotate towards a target we see
            if (gamepad1.y) {
                if (webcam.target_position != null && now - webcam.target_position.time_ms < CAMERA_UPDATE_DELAY_MS + 10) {

                    if (target_to_rotate_to == null || !target_to_rotate_to.partial_eq(webcam.target_position)) {
                        target_to_rotate_to = webcam.target_position;
                        // Math.PI / 2.0 here because current heading 0 is forward
                        wanted_heading_for_target = drivetrain.getCurrentHeading() + Math.PI / 2.0 + target_to_rotate_to.angle_distance_rads;
                    }

                    drivetrain.wanted_heading = wanted_heading_for_target;
                    ledIndicator.setPosition(LedIndicator.GREEN_POSITION);
                } else {
                    ledIndicator.setPosition(LedIndicator.YELLOW_POSITION);
                }
            }
            else if (gamepad1.yWasReleased()) {
                ledIndicator.setPosition(LedIndicator.OFF_POSITION);
            }
            else {
                if (target_to_rotate_to != null) {
                    target_to_rotate_to = null;
                }
            }

            // Enable / disable the shooter
			if (gamepad1.xWasPressed()) {
				if (shooter.flywheel_enabled) {
					shooter.update_flywheel_rpm(0.0);
                    ledIndicator.setPosition(LedIndicator.OFF_POSITION);
				} else {
                    if (target_to_rotate_to != null) {
                        shooter.update_rpm_for_distance_m(target_to_rotate_to.distance_m);
                    } else if (webcam.target_position != null && now - webcam.target_position.time_ms < 5000) {
                        shooter.update_rpm_for_distance_m(webcam.target_position.distance_m);
                    } else {
                        // Screw it
                        shooter.update_flywheel_rpm(3600.0);
                    }
				}
			}

			if (gamepad1.right_trigger > 0.1) {
                shooter.feed_ball_into_shooter();
			} else if (gamepad1.left_trigger > 0.1) {
                shooter.reset_shooter_pusher();
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

            if (now - last_updated_camera_ms > CAMERA_UPDATE_DELAY_MS) {
                last_updated_camera_ms = now;
                webcam.update();
            }

            if (webcam.target_position != null) {
                telemetry.addData("Target distance (m)", webcam.target_position.distance_m);
                telemetry.addData("Target ideal angle", Math.toDegrees(webcam.target_position.ideal_angle_to_target));
                telemetry.addData("Target to turn", Math.toDegrees(webcam.target_position.angle_distance_rads));
            }

			telemetry.update();
		}
	}
}
