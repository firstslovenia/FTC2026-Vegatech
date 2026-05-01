package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

@TeleOp(name = "Shooter++ Testing", group = "Testing")
public class ShooterPlusPlusTesting extends LinearOpMode {

    Hardware hardware;
    ShooterPlusPlus shooter;

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(100);

        hardware = new Hardware(this);
        hardware.init();

        shooter = new ShooterPlusPlus(this, hardware, null);

        waitForStart();

        shooter.run();

        while (opModeIsActive()) {
            shooter.update();

            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            telemetry.addData("RPMs (a)", shooter.last_a_rpm_measurements.average().orElse(0.0));
            telemetry.addData("RPMs (b)", shooter.last_b_rpm_measurements.average().orElse(0.0));
            telemetry.addData("Calculated RPMs", shooter.calculate_rpm(shooter.flywheel_power, voltage));
            telemetry.addData("Power    ", shooter.flywheel_power);
            telemetry.addData("Power (a)", hardware.shooterMotorA.getPower());
            telemetry.addData("Power (b)", hardware.shooterMotorB.getPower());
            telemetry.update();
        }
    }
}
