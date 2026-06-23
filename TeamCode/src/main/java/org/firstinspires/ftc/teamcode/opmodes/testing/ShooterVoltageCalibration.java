package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;

@TeleOp(name = "Shooter++ Voltage Calibration", group = "Testing")
public class ShooterVoltageCalibration extends LinearOpMode {

    Hardware hardware;
    ShooterPlusPlus shooter;
    Spindexer spindexer;

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(100);

        hardware = new Hardware(this);
        hardware.init();

        spindexer = new Spindexer(this, hardware);

        shooter = new ShooterPlusPlus(this, hardware, spindexer);
        shooter.automatically_shoot = false;

        waitForStart();

        shooter.run();
        shooter.wanted_flywheel_rpm = 3000000;

        while (opModeIsActive()) {
            shooter.update();

            telemetry.addData("RPMs", shooter.last_a_rpm_measurements.average().orElse(0.0));
            telemetry.addData("Power", hardware.shooterMotorA.getPower());

            telemetry.update();
        }
    }
}
