package org.firstinspires.ftc.teamcode.opmodes.production.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.generic.BallColor;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.generic.Vector2D;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.ShootingAuto;

@TeleOp(name = "!! DO NOT USE !! Main (Unknown Team)", group = "Production")
public class Main extends LinearOpMode {

	Hardware hardware;
	Drivetrain drivetrain;
	LedIndicator ledIndicator;
	ShooterPlusPlus shooter;
    Spindexer spindexer;
    Webcam webcam;
	Follower pedroFollower;

    Pose2D starting_pos = new Pose2D(DistanceUnit.METER, 1.0, 1.0, AngleUnit.RADIANS, Math.PI / 2.0);

    Team team = null;

    boolean intake_enabled = false;

    /// The TargetInformation of the target we're currently rotating towards
    TargetInformation target_to_rotate_to = null;
    ShooterPositioning positioning = null;

    /// The wanted heading of the target for target_to_rotate_to
    double wanted_heading_for_target = Double.NaN;
    long last_long_loop_ms = 0;
    public boolean updated_position = false;
    static long LONG_LOOP_DELAY_MS = 100;

    double servo_position = 0.0;

    SlidingWindow<Long> last_loops_took = new SlidingWindow(50);
    long last_loop_time = System.currentTimeMillis();

    double led_position_to_set = LedIndicator.OFF_POSITION;

    /// Overriden to set what's in the spindexer
    public void spindexer_override() {

    }

    // Note: gobilda IMU ne dela dobro na nizkih napetostih!
    // also, pusti za njega kkšne 0.5 s inita pred startom

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		drivetrain = new Drivetrain(this, hardware);
		drivetrain.fieldCentricTranslation = true;
		drivetrain.fieldCentricRotation = true;
		drivetrain.keepHeading = true;

		ledIndicator = new LedIndicator(this, hardware);
		led_position_to_set = LedIndicator.OFF_POSITION;

        spindexer = new Spindexer(this, hardware);

		shooter = new ShooterPlusPlus(this, hardware, spindexer);

		webcam = new Webcam(this, hardware);

		waitForStart();
        drivetrain.setPos(new Pose2D(DistanceUnit.INCH, 40, 40, AngleUnit.RADIANS, Math.PI / 2));
        drivetrain.resetStartingDirection();

        spindexer.init();
        spindexer_override();
        spindexer.switch_to_holding_pattern();

		//pedroFollower = Constants.createFollower(hardwareMap);
		//pedroFollower.update();

		while (opModeIsActive()) {

            //telemetry.addData("order", webcam.order);

            /*if (webcam.last_calculated_pos != null) {
                telemetry.addData("calculated pos x", webcam.last_calculated_pos.getX(DistanceUnit.INCH));
                telemetry.addData("calculated pos y", webcam.last_calculated_pos.getY(DistanceUnit.INCH));
                telemetry.addData("calculated angle", webcam.last_calculated_pos.getHeading(AngleUnit.DEGREES));
            }*/

            long now = System.currentTimeMillis();

            boolean do_long_loop = (now - last_long_loop_ms) >= LONG_LOOP_DELAY_MS;

            led_position_to_set = LedIndicator.OFF_POSITION;

            // If shooter is on, update LED
            if (shooter.flywheel_enabled) {

                double rpm_error = shooter.get_rpm_error();

                if (Double.isNaN(shooter.shooting_distance_m)) {
                    if (rpm_error > Shooter.SHOOTER_RPM_STABLE_ERROR_RANGE) {
                        led_position_to_set = LedIndicator.ORANGE_POSITION;
                    } else {
                        led_position_to_set = LedIndicator.YELLOW_POSITION;
                    }
                } else {
                    if (rpm_error > Shooter.SHOOTER_RPM_STABLE_ERROR_RANGE) {
                        led_position_to_set = LedIndicator.RED_POSITION;
                    } else {
                        /*if (spindexer.ball_in_shooter != null) {
                            BallColor color = spindexer.balls[spindexer.ball_in_shooter];

                            if (color == BallColor.Green) {
                                led_position_to_set = LedIndicator.GREEN_POSITION;
                            } else if (color == BallColor.Purple) {
                                led_position_to_set = LedIndicator.INDIGO_POSITION;
                            } else {
                                led_position_to_set = LedIndicator.ORANGE_POSITION;
                            }
                        } else {
                            led_position_to_set = LedIndicator.LIGHT_GREEN_POSITION;
                        }*/
                    }

                    telemetry.addData("Shooter distance (cm)", shooter.shooting_distance_m * 100.0);
                }

                telemetry.addData("Shooter RPM", shooter.wanted_flywheel_rpm);
                telemetry.addData("Shooter Error", rpm_error);
                telemetry.addData("Shooter RPM measurements len", shooter.last_a_rpm_measurements.length());
            }

			Vector2D translation_vector = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
			Vector2D rotation_vector = new Vector2D(gamepad1.right_stick_x, -gamepad1.right_stick_y);

            // Slow movement
			if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
				translation_vector = new Vector2D(0.0, 0.0);

				if (gamepad1.dpad_up) { translation_vector.y += 0.65; }
				if (gamepad1.dpad_down) { translation_vector.y -= 0.65; }
				if (gamepad1.dpad_right) { translation_vector.x += 0.65; }
				if (gamepad1.dpad_left) { translation_vector.x -= 0.65; }
			}

            // Enable / disable intake
            if (gamepad1.xWasPressed()) {
                intake_enabled = !intake_enabled;

                if (intake_enabled) {
                    hardware.intakeMotor.setPower(0.6);
                } else {
                    hardware.intakeMotor.setPower(0.0);
                }
            }

            if (gamepad1.bWasPressed()) {
                if (intake_enabled && hardware.intakeMotor.getPower() > 0.5) {
                    hardware.intakeMotor.setPower(-0.6);
                } else {
                    intake_enabled = !intake_enabled;

                    if (intake_enabled) {
                        hardware.intakeMotor.setPower(-0.6);
                    } else {
                        hardware.intakeMotor.setPower(0.0);
                    }
                }
            }

			// Reset robot to 90 degrees
			if (gamepad1.a) {
				//drivetrain.wanted_heading = Math.PI / 2.0;
                drivetrain.wanted_heading = -Math.PI / 2.0;
			}

            // Set robot to goal's angle
            if (team == null) {
                if (gamepad1.left_trigger > 0.1) {
                    drivetrain.wanted_heading = Math.toRadians(90.0 - 36.0);
                } else if (gamepad1.right_trigger > 0.1) {
                    drivetrain.wanted_heading = Math.toRadians(90.0 + 36.0);
                }
            } else {
                if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                    if (team == Team.Blue) {
                        drivetrain.wanted_heading = Math.toRadians(90.0 - 36.0);
                    } else if (team == Team.Red) {
                        drivetrain.wanted_heading = Math.toRadians(90.0 + 36.0);
                    }
                }
            }

            // Rotate towards the goal
            if (gamepad1.y || gamepad1.right_bumper) {
                positioning = new ShooterPositioning();
                target_to_rotate_to = positioning.compute_target_information(drivetrain.last_robot_position, team);
                wanted_heading_for_target = drivetrain.getCurrentHeading() + target_to_rotate_to.angle_distance_rads;
                drivetrain.wanted_heading = wanted_heading_for_target;
                led_position_to_set = LedIndicator.AQUA_POSITION;

                telemetry.addData("Target distance (m)", target_to_rotate_to.distance_m);
                telemetry.addData("Target ideal angle", Math.toDegrees(target_to_rotate_to.ideal_angle_to_target));
                telemetry.addData("Target y diff", target_to_rotate_to.y);
                telemetry.addData("Target x diff", target_to_rotate_to.x);
                telemetry.addData("Target to turn", Math.toDegrees(target_to_rotate_to.angle_distance_rads));
                telemetry.addData("Target aaaa", Math.toDegrees(target_to_rotate_to.angle_distance_to_zero));
            }
            else {
                if (target_to_rotate_to != null) {
                    target_to_rotate_to = null;
                }
            }

            if (gamepad1.guideWasPressed()) {
                drivetrain.resetStartingDirection();
            }

            // User 2
            // Enable / disable the shooter
			if (gamepad2.xWasPressed()) {
				if (shooter.flywheel_enabled) {
					shooter.update_flywheel_rpm(0.0);
				} else {
                    shooter.update_flywheel_rpm(3000.0);
                } /*else {
                    if (webcam.target_position != null && now - webcam.target_position.time_ms < 5000) {
                        shooter.update_for_target(webcam.target_position);
                    }
                    else if (target_to_rotate_to != null) {
                        shooter.update_for_target(target_to_rotate_to);
                    } else if (webcam.target_position != null && now - webcam.target_position.time_ms < 10000) {
                        shooter.update_for_target(webcam.target_position);
                    } else {
                        // Screw it
                        shooter.update_flywheel_rpm(3000.0);
                    }*/
			}
            if (gamepad2.dpadLeftWasPressed()) {
                shooter.wanted_flywheel_rpm = 2000.0;
            }
            if (gamepad2.dpadUpWasPressed()) {
                shooter.wanted_flywheel_rpm = 3000.0;
            }
            if (gamepad2.dpadRightWasPressed()) {
                shooter.wanted_flywheel_rpm = 4000.0;
            }
            if (gamepad2.dpadDownWasPressed()) {
                shooter.wanted_flywheel_rpm = 5000.0;
            }

            // Angle
            if (gamepad2.leftBumperWasPressed()) {
                servo_position -= 0.1;
            } else if (gamepad2.rightBumperWasPressed()) {
                servo_position += 0.1;
            }

            servo_position = Math.max(Math.min(servo_position, 1.0), 0.0);

            hardware.shooterAngleServo.setPosition(servo_position);

            double servo_angle = ShooterPlusPlus.calculate_rad_angle_for_servo_pos(servo_position);
            double dist_factor = ShooterPlusPlus.calculate_dist_factor_for_angle(servo_angle);

            telemetry.addData("Angle servo pos", servo_position);
            telemetry.addData("Angle servo deg", Math.toDegrees(servo_angle));
            telemetry.addData("Angle servo factor", dist_factor);

            // Fire balls
			if (gamepad2.right_trigger > 0.1) {
                hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardware.spindexerMotor.setPower(-0.5);
			} else if (hardware.spindexerMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardware.spindexerMotor.setTargetPosition(hardware.spindexerMotor.getCurrentPosition());
                spindexer.switch_to_holding_pattern();
            }

            shooter.update();

            // Not recommended!
            if (gamepad2.guideWasPressed()) {
                updated_position = false;
            }

            // Go to spindexer holding / intake (out of ex. shooting)
            if (gamepad2.yWasPressed()) {
                spindexer.switch_to_holding_pattern();
            }

            // Reset spindexer
            if (gamepad2.bWasPressed()) {
                spindexer.ball_to_intake = null;
                spindexer.ball_in_shooter = null;
                spindexer.in_survey = false;
                spindexer.move_to_angle_sortwise(Spindexer.STARTING_ANGLE);
            }

            // Run spindexer survey
            if (gamepad2.aWasPressed()) {
                spindexer.start_survey();
            }

            spindexer.update();

			// Also works for positioning!
			//
			//follower.updateConstants();
			//follower.updatePose();
			//follower.updateDrivetrain();
			//
			// Just do not use setTeleopDrive
            // Not needed currently
			//pedroFollower.update();

			drivetrain.update(translation_vector, rotation_vector);

            if (drivetrain.set_new_pos_at != 0 || drivetrain.started_compass_recalibration_ms != 0) {
                led_position_to_set = LedIndicator.ORANGE_POSITION;
            }

            if (do_long_loop) {
                webcam.update();

                /*if (!updated_position && webcam.last_calculated_pos != null && now - webcam.last_calculated_pos_time < 100) {
                    if (!ShooterPositioning.pose2d_rougly_eq(drivetrain.last_robot_position, webcam.last_calculated_pos) && now - webcam.same_pos_since > 500) {
                        drivetrain.setPos(webcam.last_calculated_pos);
                        webcam.last_calculated_pos_time = 0;
                        updated_position = true;
                    }
                }*/

                last_long_loop_ms = now;
            }

            long elapsed_ms = now - last_loop_time;
            last_loop_time = now;
            last_loops_took.push(elapsed_ms);

            if (last_loops_took.average().isPresent()) {
                telemetry.addData("Last 50 loop avg", last_loops_took.average().get());
            }

            hardware.rgbLed.setPosition(led_position_to_set);

            telemetry.addData("RPM (measured)", shooter.last_a_rpm_measurements.average().orElse(0.0));
            telemetry.update();
		}
	}
}
