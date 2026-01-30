package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

	/// Constant to convert cm to inches
	public static double CM_TO_INCH = 1 / 2.54;
	public static FollowerConstants followerConstants = new FollowerConstants().mass(8.23).forwardZeroPowerAcceleration(-63.259588988639).lateralZeroPowerAcceleration(-72.55380727727302);

	public static MecanumConstants driveConstants = new MecanumConstants()
		.maxPower(1)
		.rightFrontMotorName("frontRightMotor")
		.rightRearMotorName("backRightMotor")
		.leftFrontMotorName("frontLeftMotor")
		.leftRearMotorName("backLeftMotor")
		.leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
		.leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
		.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
		.rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

	public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
		.forwardTicksToInches(9.092275468167541E-4)
		.strafeTicksToInches(9.376179748759496E-4)
		.turnTicksToInches(0.002000318108820521)
		.leftPodY(17.73 * CM_TO_INCH)
		.rightPodY(-15.77 * CM_TO_INCH)
		.strafePodX(-18.83 * CM_TO_INCH)
		.leftEncoder_HardwareMapName("frontLeftMotor")
		.rightEncoder_HardwareMapName("backRightMotor")
		.strafeEncoder_HardwareMapName("backLeftMotor")
		.leftEncoderDirection(Encoder.REVERSE)
		.rightEncoderDirection(Encoder.REVERSE)
		//.strafeEncoderDirection(Encoder.REVERSE)
		.IMU_HardwareMapName("imu")
		.IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
			.pathConstraints(pathConstraints)
			.mecanumDrivetrain(driveConstants)
			.threeWheelIMULocalizer(localizerConstants)
			.build();
	}
}
