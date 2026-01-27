package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Hardware {

	private OpMode callingOpMode = null;

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

	public static DcMotor leftForwardDeadwheel = null;
	public static DcMotor rightForwardDeadwheel = null;
	public static DcMotor backSidewaysDeadwheel = null;

	public static DcMotor shooterMotor = null;
	public static Servo shooterPusherServo = null;

    public static DcMotor intakeMotor = null;
    public static DcMotor spindexerMotor = null;
	public static IMU imu = null;

	public static Servo rgbLed = null;

	public static WebcamName webcam = null;

	public Hardware (OpMode opmode) {
		callingOpMode = opmode;
	}

	public void init()    {
		frontLeftMotor = callingOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
		frontRightMotor = callingOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
		backLeftMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
		backRightMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");

		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Make the positive direction on all motors +y
		// (This is assuming the motors spin clockwise and their directions aren't flipped by gears)
		frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		backSidewaysDeadwheel = callingOpMode.hardwareMap.get(DcMotor.class, "backSidewaysDeadwheel");
		leftForwardDeadwheel = callingOpMode.hardwareMap.get(DcMotor.class, "leftForwardDeadwheel");
		rightForwardDeadwheel = callingOpMode.hardwareMap.get(DcMotor.class, "rightForwardDeadwheel");

		shooterMotor = callingOpMode.hardwareMap.get(DcMotor.class, "shooterMotor");
		shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		shooterPusherServo = callingOpMode.hardwareMap.get(Servo.class, "shooterPusherServo");
        shooterPusherServo.setDirection(Servo.Direction.REVERSE);

        // Hack: they are on the same port
        intakeMotor = callingOpMode.hardwareMap.get(DcMotor.class, "backSidewaysDeadwheel");
        spindexerMotor = callingOpMode.hardwareMap.get(DcMotor.class, "leftForwardDeadwheel");

		imu = callingOpMode.hardwareMap.get(IMU.class, "imu");
		imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

		rgbLed = callingOpMode.hardwareMap.get(Servo.class, "rgbLed");
		webcam = callingOpMode.hardwareMap.get(WebcamName.class, "webcam");
	}
}
