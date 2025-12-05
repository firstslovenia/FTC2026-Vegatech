package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;

@TeleOp(name = "Shooter Testing")
public class ShooterTesting extends LinearOpMode {

	Hardware hardware;
	LedIndicator ledIndicator;
	Shooter shooter;

	double rpms_x100 = 20.0;

	@Override
	public void runOpMode() {

		hardware = new Hardware(this);
		hardware.init();

		ledIndicator = new LedIndicator(this, hardware);

		shooter = new Shooter(this, hardware);

		waitForStart();

		while (opModeIsActive()) {
			if (gamepad1.aWasPressed()) {
				rpms_x100 += 1.0;
			}

			if (gamepad1.bWasPressed()) {
				rpms_x100 -= 1.0;
			}

			shooter.update_flywheel_rpm(rpms_x100 * 100.0);
			shooter.update();

			if (Math.abs(rpms_x100 * 100.0 - shooter.last_rpm_measurements.average().orElse(0.0)) > 100.0) {
				ledIndicator.setPosition(LedIndicator.RED_POSITION);
			} else if (Math.abs(rpms_x100 * 100.0 - shooter.last_rpm_measurements.average().orElse(0.0)) > 20.0) {
				ledIndicator.setPosition(LedIndicator.YELLOW_POSITION);
			} else if (shooter.flywheel_enabled) {
				ledIndicator.setPosition(LedIndicator.GREEN_POSITION);
			} else {
				ledIndicator.setPosition(LedIndicator.OFF_POSITION);
			}

			telemetry.addData("rpms x100", rpms_x100);
			telemetry.addData("RPM (measured)", shooter.last_rpm_measurements.average().orElse(0.0));
			telemetry.addData("Powah", hardware.shooterMotor.getPower());
			telemetry.addData("Slow start multiplier", shooter.last_slow_start_multiplier);
			telemetry.addData("Encoder pos", shooter.last_position_ticks.average().orElse(0.0));
			telemetry.update();


			telemetry.addData("power", rpms_x100);
			telemetry.addData("RPM (measured)", shooter.last_rpm_measurements.average().orElse(0.0));
			telemetry.update();
		}
	}
}
