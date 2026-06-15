package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;

@TeleOp(name = "Hardware Testing", group = "Testing")
public class HardwareTesting extends LinearOpMode {

    Hardware hardware;
    Spindexer spindexer;

    int spindexer_pos = 0;

    HardwareComponent component = HardwareComponent.frontLeftMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new Hardware(this);
        hardware.init();

        spindexer = new Spindexer(this, hardware);
        spindexer.init();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                component = component.next();
            }
            if (gamepad1.bWasPressed()) {
                component = component.previous();
            }

            telemetry.addData("Component", component);

            double power = -gamepad1.right_stick_y;

            switch (component) {
                case frontLeftMotor:
                    hardware.frontLeftMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case frontRightMotor:
                    hardware.frontRightMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case backLeftMotor:
                    hardware.backLeftMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case backRightMotor:
                    hardware.backRightMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case shooterMotor:
                    hardware.shooterMotorA.setPower(power);
                    hardware.shooterMotorB.setPower(power);
                    telemetry.addData("Power", power);
                    telemetry.addData("A ticks", hardware.shooterMotorA.getCurrentPosition());
                    telemetry.addData("B ticks", hardware.shooterMotorB.getCurrentPosition());
                    break;
                case intakeMotor:
                    hardware.intakeMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case spindexerMotor:

                    hardware.spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    hardware.spindexerMotor.setPower(power);

                    telemetry.addData("Power", power);

                    int pos = hardware.spindexerMotor.getCurrentPosition();
                    int pos_in_loop = Spindexer.calculate_relative_loop_ticks(pos);

                    telemetry.addData("Position", pos);
                    telemetry.addData("Ticks in loop", pos_in_loop);
                    telemetry.addData("Nearest 0 angle", Spindexer.nearest_shootwise_pos_at_angle(pos, 0.0));
                    telemetry.addData("Angle", Math.toDegrees(Spindexer.calculate_current_angle(pos)));
                    break;

                case Spindexer:

                    int pos_2 = hardware.spindexerMotor.getCurrentPosition();
                    int pos_in_loop_2 = Spindexer.calculate_relative_loop_ticks(pos_2);

                    if (gamepad1.yWasPressed()) {
                        spindexer_pos += 1;
                        spindexer_pos = Math.floorMod(spindexer_pos, 7);
                    } else if (gamepad1.xWasPressed()) {
                        spindexer_pos -= 1;
                        spindexer_pos = Math.floorMod(spindexer_pos, 7);
                    }

                    if (gamepad1.left_bumper) {
                        switch (spindexer_pos) {
                            case 0:
                                spindexer.move_to_angle_sortwise(0.0);
                                break;
                            case 1:
                                spindexer.move_to_angle_sortwise(Spindexer.ANGLE_INTAKE_BALL_2);
                                break;
                            case 2:
                                spindexer.move_to_angle_sortwise(Spindexer.ANGLE_INTAKE_BALL_1);
                                break;
                            case 3:
                                spindexer.move_to_angle_sortwise(Spindexer.ANGLE_INTAKE_BALL_0);
                                break;
                            case 4:
                                spindexer.move_to_angle_sortwise(Spindexer.ANGLE_SHOOT_BALL_2);
                                break;
                            case 5:
                                spindexer.move_to_angle_sortwise(Spindexer.ANGLE_SHOOT_BALL_1);
                                break;
                            case 6:
                                spindexer.move_to_angle_sortwise(Spindexer.ANGLE_SHOOT_BALL_0);
                                break;
                        }
                    }

                    if (gamepad1.rightTriggerWasPressed()) {
                        if (spindexer.ball_in_shooter != null) {
                            spindexer.shoot_active_ball();
                        } else {
                            spindexer.move_to_shoot_ball(0);
                        }
                    }

                    telemetry.addData("Preset Position", spindexer_pos);
                    telemetry.addData("Position", pos_2);
                    telemetry.addData("Ticks in loop", pos_in_loop_2);
                    telemetry.addData("Nearest 0 angle", Spindexer.nearest_sortwise_pos_at_angle(pos_2, 0.0));
                    telemetry.addData("Angle", Math.toDegrees(spindexer.current_angle()));
                    telemetry.addData("Target Angle", Math.toDegrees(spindexer.target_angle()));
                    telemetry.addData("PID working", hardware.spindexerMotor.isBusy());

                    if (spindexer.stopped_being_busy_ms != null) {
                        telemetry.addData("Busy for", System.currentTimeMillis() - spindexer.stopped_being_busy_ms);
                    }

                    telemetry.addData("PID tolerance", ((DcMotorEx) hardware.spindexerMotor).getTargetPositionTolerance());

                    spindexer.update();

                    break;
                case odometry:
                    hardware.odometry.update();

                    Pose2D position = hardware.odometry.getPosition();

                    telemetry.addData("orientation           ", position.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("x pos (m)             ", position.getX(DistanceUnit.METER));
                    telemetry.addData("y pos (m)             ", position.getY(DistanceUnit.METER));

                    break;
                case cameraAngleServo:

                    double servo_pos = hardware.cameraAngleServo.getPosition();

                    telemetry.addData("servo position", servo_pos);

                    if (gamepad1.xWasPressed()) {
                        servo_pos = servo_pos + 0.1;
                    } else if (gamepad1.yWasPressed()) {
                        servo_pos = servo_pos - 0.1;
                    }

                    servo_pos = Math.min(Math.max(0.0, servo_pos), 1.0);

                    hardware.cameraAngleServo.setPosition(servo_pos);


                case shooterAngleServo:

                    double servo_pos_3 = hardware.shooterAngleServo.getPosition();

                    telemetry.addData("servo position", servo_pos_3);

                    if (gamepad1.xWasPressed()) {
                        servo_pos_3 = servo_pos_3 + 0.1;
                    } else if (gamepad1.yWasPressed()) {
                        servo_pos_3 = servo_pos_3 - 0.1;
                    }

                    servo_pos_3 = Math.min(Math.max(0.0, servo_pos_3), 1.0);

                    double servo_angle = ShooterPlusPlus.calculate_rad_angle_for_servo_pos(servo_pos_3);
                    double dist_factor = ShooterPlusPlus.calculate_dist_factor_for_angle(servo_angle);

                    telemetry.addData("Angle servo deg", Math.toDegrees(servo_angle));
                    telemetry.addData("Angle servo factor", dist_factor);

                    hardware.shooterAngleServo.setPosition(servo_pos_3);

                case lifterServo:

                    /*double servo_pos_2 = hardware.lifterServo.getPosition();

                    telemetry.addData("servo position", servo_pos_2);

                    if (gamepad1.xWasPressed()) {
                        servo_pos_2 = servo_pos_2 + 0.1;
                    } else if (gamepad1.yWasPressed()) {
                        servo_pos_2 = servo_pos_2 - 0.1;
                    }

                    servo_pos_2 = Math.min(Math.max(0.0, servo_pos_2), 1.0);

                    hardware.lifterServo.setPosition(servo_pos_2);*/

                case rgbLed:
                    double color = LedIndicator.RED_POSITION + (LedIndicator.VIOLET_POSITION - LedIndicator.RED_POSITION) * ((double)(System.currentTimeMillis() % 10000) / 10000.0);

                    hardware.rgbLed.setPosition(color);
                    telemetry.addData("Position", color);
                    break;
                case colorSensor:
                    NormalizedRGBA output = hardware.colorSensor.getNormalizedColors();
                    double distance = hardware.colorSensor.getDistance(DistanceUnit.CM);

                    telemetry.addData("Distance [cm]", distance);
                    telemetry.addData("R", output.red);
                    telemetry.addData("G", output.green);
                    telemetry.addData("B", output.blue);
                    telemetry.addData("A", output.alpha);
            }

            telemetry.update();
        }
    }
}
