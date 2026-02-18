package org.firstinspires.ftc.teamcode.opmodes.production.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.generic.BallColor;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Shooter Power Testing", group = "Production")
public class ShooterPowerTesting extends LinearOpMode {

    Hardware hardware;
    Drivetrain drivetrain;
    LedIndicator ledIndicator;
    Shooter shooter;
    Spindexer spindexer;
    Webcam webcam;
    Follower pedroFollower;

    boolean intake_enabled = false;

    double wanted_power_rpms = 3000.0;

    /// The TargetInformation of the target we're currently rotating towards
    TargetInformation target_to_rotate_to = null;
    /// The wanted heading of the target for target_to_rotate_to
    double wanted_heading_for_target = Double.NaN;
    long last_long_loop_ms = 0;
    static long LONG_LOOP_DELAY_MS = 100;

    double led_position_to_set = LedIndicator.OFF_POSITION;

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

        shooter = new Shooter(this, hardware);
        shooter.reset_shooter_pusher();

        spindexer = new Spindexer(this, hardware);

        webcam = new Webcam(this, hardware);

        waitForStart();
        drivetrain.resetStartingDirection();

        spindexer.init();
        spindexer.switch_to_holding_pattern();

        pedroFollower = Constants.createFollower(hardwareMap);
        pedroFollower.update();

        while (opModeIsActive()) {

            telemetry.addData("Wanted RPMs", wanted_power_rpms);
            telemetry.addData("RPMs", shooter.last_rpm_measurements.average().orElse(0.0));
            telemetry.addData("Past startup", shooter.past_startup ? 1 : 0);

            if (webcam.target_position != null) {
                telemetry.addData("Distance [m]", webcam.target_position.tag_distance_m);
            }

            telemetry.addLine("");

            long now = System.currentTimeMillis();

            boolean do_long_loop = (now - last_long_loop_ms) >= LONG_LOOP_DELAY_MS;

            led_position_to_set = LedIndicator.OFF_POSITION;

            // If spindexer is on, update LED
            if (spindexer.ball_to_intake != null) {
                led_position_to_set = LedIndicator.VIOLET_POSITION;
            }

            if (spindexer.ball_in_shooter != null) {
                BallColor color = spindexer.balls[spindexer.ball_in_shooter];

                if (color == BallColor.Green) {
                    led_position_to_set = LedIndicator.GREEN_POSITION;
                } else if (color == BallColor.Purple) {
                    led_position_to_set = LedIndicator.INDIGO_POSITION;
                } else {
                    led_position_to_set = LedIndicator.ORANGE_POSITION;
                }
            }

            // If shooter is on, update LED
            if (shooter.flywheel_enabled) {

                double rpm_error = shooter.get_rpm_error();

                if (Double.isNaN(shooter.shooting_distance_m)) {
                    if (rpm_error > Shooter.SHOOTER_RPM_SEMI_STABLE_ERROR_RANGE) {
                        led_position_to_set = LedIndicator.ORANGE_POSITION;
                    } else if (rpm_error > Shooter.SHOOTER_RPM_STABLE_ERROR_RANGE) {
                        led_position_to_set = LedIndicator.BLUE_POSITION;
                    } else {
                        led_position_to_set = LedIndicator.INDIGO_POSITION;
                    }
                } else {
                    if (rpm_error > Shooter.SHOOTER_RPM_SEMI_STABLE_ERROR_RANGE) {
                        led_position_to_set = LedIndicator.RED_POSITION;
                    } else if (rpm_error > Shooter.SHOOTER_RPM_STABLE_ERROR_RANGE) {
                        led_position_to_set = LedIndicator.YELLOW_POSITION;
                    } else {
                        led_position_to_set = LedIndicator.GREEN_POSITION;
                    }

                    telemetry.addData("Shooter distance (cm)", shooter.shooting_distance_m * 100.0);
                }

                telemetry.addData("Shooter RPM", shooter.wanted_flywheel_rpm);
                telemetry.addData("Shooter Error", rpm_error);
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
                    hardware.intakeMotor.setPower(1.0);
                } else {
                    hardware.intakeMotor.setPower(0.0);
                }
            }

            if (gamepad1.bWasPressed()) {
                if (intake_enabled && hardware.intakeMotor.getPower() > 0.9) {
                    hardware.intakeMotor.setPower(-1.0);
                } else {
                    intake_enabled = !intake_enabled;

                    if (intake_enabled) {
                        hardware.intakeMotor.setPower(-1.0);
                    } else {
                        hardware.intakeMotor.setPower(0.0);
                    }
                }
            }

            // Reset robot to 90 degrees
            if (gamepad1.a) {
                drivetrain.wanted_heading = Math.PI / 2.0;
            }

            // Rotate towards a target we see
            if (gamepad1.y) {

                if (do_long_loop) {
                    webcam.update();
                }

                if (webcam.target_position != null && now - webcam.target_position.time_ms < LONG_LOOP_DELAY_MS + 10) {

                    if (target_to_rotate_to == null || !target_to_rotate_to.partial_eq(webcam.target_position)) {
                        target_to_rotate_to = webcam.target_position;
                        // Math.PI / 2.0 here because current heading 0 is forward
                        wanted_heading_for_target = drivetrain.getCurrentHeading() + Math.PI / 2.0 + target_to_rotate_to.angle_distance_rads;
                    }

                    drivetrain.wanted_heading = wanted_heading_for_target;
                    led_position_to_set = LedIndicator.GREEN_POSITION;
                } else {
                    led_position_to_set = LedIndicator.YELLOW_POSITION;
                }
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

            if (gamepad2.dpadUpWasPressed()) {
                wanted_power_rpms += 100.0;
            } else if (gamepad2.dpadDownWasPressed()) {
                wanted_power_rpms -= 100.0;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                wanted_power_rpms -= 10.0;
            } else if (gamepad2.dpadRightWasPressed()) {
                wanted_power_rpms += 10.0;
            }

            // Enable / disable the shooter
            if (gamepad2.xWasPressed()) {
                if (shooter.flywheel_enabled) {
                    shooter.update_flywheel_rpm(0.0);
                } else {
                    shooter.update_flywheel_rpm(wanted_power_rpms);
                }
            }

            if (gamepad2.aWasPressed()) {
                if (shooter.flywheel_enabled) {
					shooter.update_flywheel_rpm(0.0);
				} else {
                    if (target_to_rotate_to != null) {
                        shooter.update_rpm_for_distance_m(target_to_rotate_to.distance_m);
                    } else if (webcam.target_position != null && now - webcam.target_position.time_ms < 10000) {
                        shooter.update_rpm_for_distance_m(webcam.target_position.distance_m);
                    }
				}
            }

            // Fire balls
            if (gamepad2.right_trigger > 0.1) {
                shooter.fire();

                if (spindexer.ball_in_shooter != null) {
                    spindexer.balls[spindexer.ball_in_shooter] = BallColor.None;
                    spindexer.ball_in_shooter = null;
                }
            }

            shooter.update();

            // Select spindexer ball
            if (gamepad2.leftBumperWasPressed()) {
                if (spindexer.ball_in_shooter != null && spindexer.balls[spindexer.ball_in_shooter] == BallColor.Green) {
                    spindexer.switch_to_holding_pattern();
                } else {
                    spindexer.switch_to_coloured_ball(BallColor.Green);
                }
            } else if (gamepad2.rightBumperWasPressed()) {
                if (spindexer.ball_in_shooter != null && spindexer.balls[spindexer.ball_in_shooter] == BallColor.Purple) {
                    spindexer.switch_to_holding_pattern();
                } else {
                    spindexer.switch_to_coloured_ball(BallColor.Purple);
                }
            }

            // Do spindexer intake
            if (gamepad2.yWasPressed()) {
                if (spindexer.ball_to_intake != null) {
                    spindexer.switch_to_holding_pattern();
                } else {
                    spindexer.switch_to_available_intake();
                }
            }

            // Reset spindexer
            if (gamepad2.bWasPressed()) {
                spindexer.ball_to_intake = null;
                spindexer.ball_in_shooter = null;
                spindexer.move_to_angle(0.0);
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

            if (webcam.target_position != null) {
                telemetry.addData("Target distance (m)", webcam.target_position.distance_m);
                telemetry.addData("Target ideal angle", Math.toDegrees(webcam.target_position.ideal_angle_to_target));
                telemetry.addData("Target to turn", Math.toDegrees(webcam.target_position.angle_distance_rads));
            }

            if (do_long_loop) {
                last_long_loop_ms = now;
            }

            hardware.rgbLed.setPosition(led_position_to_set);
            telemetry.update();
        }
    }
}

