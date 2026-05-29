package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

@TeleOp(name = "Shooter++ Testing", group = "Testing")
public class ShooterPlusPlusTesting extends LinearOpMode {

    Hardware hardware;
    ShooterPlusPlus shooter;
    Webcam webcam;

    double servo_position = 0.0;

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(100);

        hardware = new Hardware(this);
        hardware.init();

        shooter = new ShooterPlusPlus(this, hardware, null);

        webcam = new Webcam(this, hardware);

        waitForStart();

        shooter.run();

        while (opModeIsActive()) {
            shooter.update();
            webcam.update();

            if (gamepad1.aWasPressed()) {
                servo_position = Math.max(0.0, servo_position - 0.1);
            } else if (gamepad1.bWasPressed()) {
                servo_position = Math.min(1.0, servo_position + 0.1);
            }
            hardware.shooterAngleServo.setPosition(servo_position);

            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            telemetry.addData("Angle servo pos", hardware.shooterAngleServo.getPosition());
            telemetry.addData("RPMs (a)", shooter.last_a_rpm_measurements.average().orElse(0.0));
            telemetry.addData("Calculated RPMs", shooter.calculate_rpm(shooter.flywheel_power, voltage));
            telemetry.addData("Power    ", shooter.flywheel_power);
            telemetry.addData("Power (a)", hardware.shooterMotorA.getPower());

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
