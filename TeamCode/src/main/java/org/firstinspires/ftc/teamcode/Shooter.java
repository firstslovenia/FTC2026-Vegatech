package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Shooter {
	LinearOpMode callingOpMode;
	Hardware hardware;

	public boolean enabled = false;

	public Shooter(LinearOpMode callingOpMode, Hardware hardware) {
		hardware.shooterMotor.setPower(0.0);
	}

	public void update(double power) {
		enabled = power != 0.0;
		hardware.shooterMotor.setPower(power);
	}
}
