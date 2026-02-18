package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

@TeleOp(name = "Shooter Testing", group = "Testing")
public class ShooterTesting extends LinearOpMode {

	Hardware hardware;
	LedIndicator ledIndicator;
	Shooter shooter;
	Webcam webcam;

	//double distance_cm = 200.0;

	double rpms_x100 = 30.0;

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
                shooter.update_flywheel_rpm(rpms_x100 * 100.0);
			}

			if (gamepad1.bWasPressed()) {
				rpms_x100 -= 1.0;
                shooter.update_flywheel_rpm(rpms_x100 * 100.0);
			}

			shooter.update();

			if (Math.abs(shooter.wanted_flywheel_rpm - shooter.last_rpm_measurements.average().orElse(0.0)) > 100.0) {
				ledIndicator.setPosition(LedIndicator.RED_POSITION);
			} else if (Math.abs(shooter.wanted_flywheel_rpm - shooter.last_rpm_measurements.average().orElse(0.0)) > 20.0) {
				ledIndicator.setPosition(LedIndicator.YELLOW_POSITION);
			} else if (shooter.flywheel_enabled) {
				ledIndicator.setPosition(LedIndicator.GREEN_POSITION);
			} else {
				ledIndicator.setPosition(LedIndicator.OFF_POSITION);
			}

			telemetry.addData("Wanted RPMs", rpms_x100 * 100.0);
			telemetry.addData("RPMs", shooter.last_rpm_measurements.average().orElse(0.0));
            telemetry.addData("Past startup", shooter.past_startup ? 1 : 0);
			telemetry.addData("Powah", hardware.shooterMotor.getPower());
			telemetry.addData("Encoder pos", shooter.last_position_ticks.average().orElse(0.0));
			telemetry.update();
			telemetry.addData("power", rpms_x100);

            /*iif (last.average().isPresent()) {
                telemetry.addData("Last 50 loop avg", shooter.last_loops_took.average().get());
            }*/

            telemetry.update();
		}
	}
}
