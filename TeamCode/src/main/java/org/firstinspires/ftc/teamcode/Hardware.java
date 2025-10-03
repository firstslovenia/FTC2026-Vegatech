package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

public class Hardware {

	private LinearOpMode callingOpMode = null;

	//  Motor configuration:
	//  ╔════════════════════╗
	//  ║ \     front     /  ║
	//  ║ \  left  right  /  ║
	//  ║ \               /  ║
	//  ║                    ║
	//  ║ /     back      \  ║
	//  ║ /  left  right  \  ║
	//  ║ /               \  ║
	//  ╚════════════════════╝

	public static DcMotor frontLeftMotor = null;
	public static DcMotor frontRightMotor = null;
	public static DcMotor backLeftMotor = null;
	public static DcMotor backRightMotor = null;

	public static IMU imu = null;

	public Hardware (LinearOpMode opmode) {
		callingOpMode = opmode;
	}

	public void init()    {
		frontLeftMotor = callingOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
		frontRightMotor = callingOpMode.hardwareMap.get(DcMotor.class, "frontRight");
		backLeftMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backLeft");
		backRightMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backRight");

		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Make the positive direction on all motors +y
		// (This is assuming the motors spin clockwise and their directions aren't flipped by gears)
		frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = callingOpMode.hardwareMap.get(IMU.class, "imu");
	}
}
