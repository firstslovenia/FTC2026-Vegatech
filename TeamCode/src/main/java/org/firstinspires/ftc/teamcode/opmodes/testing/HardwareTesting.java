package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;

@TeleOp(name = "Hardware Testing", group = "Testing")
public class HardwareTesting extends LinearOpMode {

    Hardware hardware;
    Shooter shooter;

    HardwareComponent component = HardwareComponent.frontLeftMotor;

    @Override
    public void runOpMode() {

        hardware = new Hardware(this);
        hardware.init();

        shooter = new Shooter(this, hardware);

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
                    hardware.shooterMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case intakeMotor:
                    hardware.intakeMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case spindexerMotor:
                    hardware.spindexerMotor.setPower(power);
                    telemetry.addData("Power", power);
                    break;
                case shooterPusherServo:

                    if (gamepad1.xWasPressed()) {
                        hardware.shooterPusherServo.setPosition(0.0);
                    } else if (gamepad1.yWasReleased()) {
                        hardware.shooterPusherServo.setPosition(1.0);
                    }

                    telemetry.addData("Position", hardware.shooterPusherServo.getPosition());

                    break;
                case leftForwardDeadwheel:
                    telemetry.addData("Ticks", hardware.leftForwardDeadwheel.getCurrentPosition());
                    break;
                case rightForwardDeadwheel:
                    telemetry.addData("Ticks", hardware.rightForwardDeadwheel.getCurrentPosition());
                    break;
                case backSidewaysDeadwheel:
                    telemetry.addData("Ticks", hardware.backSidewaysDeadwheel.getCurrentPosition());
                    break;
                case rgbLed:
                    double color = LedIndicator.RED_POSITION + (LedIndicator.VIOLET_POSITION - LedIndicator.RED_POSITION) * ((double)(System.currentTimeMillis() % 10000) / 10000.0);

                    hardware.rgbLed.setPosition(color);
                    telemetry.addData("Position", color);
                    break;
            }

            telemetry.update();
        }
    }
}
