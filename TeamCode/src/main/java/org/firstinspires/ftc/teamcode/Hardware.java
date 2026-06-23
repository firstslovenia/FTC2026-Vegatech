package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

	public static DcMotor shooterMotorA = null;
    public static DcMotor shooterMotorB = null;
	public static Servo shooterAngleServo = null;
	public static Servo cameraAngleServo = null;
    public static Servo lifterServo = null;

    public static DcMotor intakeMotor = null;
    public static DcMotor spindexerMotor = null;
	public static IMU imu = null;
	public static Servo rgbLed = null;
    public static RevColorSensorV3 colorSensor = null;

	public static WebcamName webcam = null;
    public static GoBildaPinpointDriver odometry = null;
    public static double ODOMETRY_EPSILON = 0.0001;

	public Hardware (OpMode opmode) {
		callingOpMode = opmode;
	}

    /// Returns true if our odometry has no starting position set
    public boolean is_odometry_starting_pos_unset() {
        return Math.abs(odometry.getPosition().getX(DistanceUnit.METER)) < ODOMETRY_EPSILON && Math.abs(odometry.getPosition().getY(DistanceUnit.METER)) < ODOMETRY_EPSILON;
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
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

		odometry = callingOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        // Keep position from auto
        if (is_odometry_starting_pos_unset()) {
            odometry.setOffsets(-15.63, -17.03, DistanceUnit.CM);
            odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odometry.resetPosAndIMU();
        }

		shooterMotorA = callingOpMode.hardwareMap.get(DcMotor.class, "shooterMotorA");
        shooterMotorA.setDirection(DcMotorSimple.Direction.REVERSE);
		shooterMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorB = callingOpMode.hardwareMap.get(DcMotor.class, "shooterMotorB");
        shooterMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 0.0 is fully forward, 1.0 is fully up
		shooterAngleServo = callingOpMode.hardwareMap.get(Servo.class, "shooterAngleServo");

        // This is measured!!! Anything more than the second value does not change the angle
        shooterAngleServo.scaleRange(0.0, 0.75);

        cameraAngleServo = callingOpMode.hardwareMap.get(Servo.class, "cameraAngleServo");

        lifterServo = callingOpMode.hardwareMap.get(Servo.class, "lifterServo");
        lifterServo.scaleRange(0.3, 1.0);
        lifterServo.setDirection(Servo.Direction.REVERSE);

        intakeMotor = callingOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexerMotor = callingOpMode.hardwareMap.get(DcMotor.class, "spindexerMotor");
        spindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		//imu = callingOpMode.hardwareMap.get(IMU.class, "imu");
		//imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

		rgbLed = callingOpMode.hardwareMap.get(Servo.class, "rgbLed");

        // Rely primarily on gobilda, use the camera to set the initial position; autonomous gets us reliable positioning anyway
        // To target stuff use calculated gobilda pos
		webcam = callingOpMode.hardwareMap.get(WebcamName.class, "webcam");

        colorSensor = callingOpMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
	}
}
