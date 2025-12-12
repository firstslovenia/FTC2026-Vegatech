package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;

@TeleOp(name = "Shooter Power Testing")
public class ShooterPowerTesting extends LinearOpMode {

	Hardware hardware;
	LedIndicator ledIndicator;
	Shooter shooter;

	double power = 0.2;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		ledIndicator = new LedIndicator(this, hardware);

		shooter = new Shooter(this, hardware);
		shooter.dry_run = true;

		waitForStart();

		while (opModeIsActive()) {
			if (gamepad1.aWasPressed()) {
				power += 0.001;
			}

			if (gamepad1.bWasPressed()) {
				power -= 0.001;
			}

			hardware.shooterMotor.setPower(power);
			shooter.update();

			telemetry.addData("power", power);
			telemetry.addData("RPM (measured)", shooter.last_rpm_measurements.average().orElse(0.0));
			telemetry.update();
		}
	}
}
