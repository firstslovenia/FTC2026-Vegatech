package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ColorOrder;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;

@TeleOp(name = "Shooter++ Testing", group = "Testing")
public class ShooterPlusPlusTesting extends LinearOpMode {

    Hardware hardware;
    ShooterPlusPlus shooter;
    Webcam webcam;
    Spindexer spindexer;

    double servo_position = 0.0;
    double rpm_x100 = 1.0;

    long last_fire = 0;
    boolean firing = false;

    boolean intake_enabled = false;

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(100);

        hardware = new Hardware(this);
        hardware.init();

        webcam = new Webcam(this, hardware);

        spindexer = new Spindexer(this, hardware);

        shooter = new ShooterPlusPlus(this, hardware, spindexer);
        shooter.automatically_shoot = false;

        waitForStart();

        spindexer.init();
        spindexer.switch_to_holding_pattern();

        shooter.run();

        while (opModeIsActive()) {
            webcam.update();

            // Fire balls
            if (gamepad1.guideWasPressed()) {

                ColorOrder order = ColorOrder.Unknown;

                if (webcam != null) {
                    order = webcam.order;
                }

                spindexer.switch_to_shooting(order);
            }

            // Go to spindexer holding / intake (out of ex. shooting)
            if (gamepad1.dpadUpWasPressed()) {
                spindexer.switch_to_holding_pattern();
            }

            // Run spindexer survey
            if (gamepad1.leftStickButtonWasPressed()) {
                spindexer.start_survey();
            }

            // Reset spindexer
            if (gamepad1.dpadDownWasPressed()) {
                spindexer.reset_state();
            }

            spindexer.update();
            shooter.update();

            if (gamepad1.aWasPressed()) {
                if (gamepad1.left_bumper) {
                    servo_position -= 0.01;
                } else {
                    servo_position -= 0.1;
                }
            } else if (gamepad1.bWasPressed()) {
                if (gamepad1.left_bumper) {
                    servo_position += 0.01;
                } else {
                    servo_position += 0.1;
                }
            }

            servo_position = Math.max(Math.min(servo_position, 1.0), 0.0);

            if (gamepad1.dpadRightWasPressed()) {
                intake_enabled = !intake_enabled;

                if (intake_enabled) {
                    hardware.intakeMotor.setPower(0.6);
                } else {
                    hardware.intakeMotor.setPower(0.0);
                }
            }

            if (gamepad1.dpadLeftWasPressed()) {
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
            if (gamepad1.xWasPressed()) {
                rpm_x100 += 1.0;
            } else if (gamepad1.yWasPressed()) {
                rpm_x100 -= 1.0;
            }

            if (rpm_x100 > 0.0 && !shooter.flywheel_enabled) {
                shooter.run();
            }

            shooter.wanted_flywheel_rpm = rpm_x100 * 100.0;

            hardware.shooterAngleServo.setPosition(servo_position);

            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double servo_angle = ShooterPlusPlus.calculate_rad_angle_for_servo_pos(servo_position);
            double dist_factor = ShooterPlusPlus.calculate_dist_factor_for_angle(servo_angle);

            telemetry.addData("Angle servo pos", servo_position);
            telemetry.addData("Angle servo deg", Math.toDegrees(servo_angle));
            telemetry.addData("Angle servo factor", dist_factor);
            telemetry.addData("Wanted RPM", shooter.wanted_flywheel_rpm);
            telemetry.addData("Wanted RPM (b)", rpm_x100);
            telemetry.addData("RPMs (a)", shooter.last_a_rpm_measurements.average().orElse(0.0));
            telemetry.addData("Calculated RPMs", shooter.calculate_rpm(shooter.flywheel_power, voltage));
            telemetry.addData("Power    ", shooter.flywheel_power);
            telemetry.addData("Power (a)", hardware.shooterMotorA.getPower());
            telemetry.addData("Shooter gain", shooter.flywheel_gain);
            telemetry.addData("Voltage [V]", voltage);

            TargetInformation target_information = webcam.target_position;

            if (target_information != null) {
                long now = System.currentTimeMillis();

                long target_info_age = now - target_information.time_ms;

                if (target_info_age < 1000) {
                    telemetry.addData("Distance [m]", target_information.tag_distance_m);
                    telemetry.addData("(Distance age) [ms]", target_info_age);
                }
            }

            telemetry.update();
        }
    }
}
