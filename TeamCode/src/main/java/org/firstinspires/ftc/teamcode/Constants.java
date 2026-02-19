package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

	/// Constant to convert cm to inches
	public static double CM_TO_INCH = 1 / 2.54;
	public static FollowerConstants followerConstants = new FollowerConstants()
            // Ish, update me please!
            .mass(10.0)
            .forwardZeroPowerAcceleration(-36.940776023051384)
            .lateralZeroPowerAcceleration(-80.24074184452753);

	public static MecanumConstants driveConstants = new MecanumConstants()
		.maxPower(1)
		.rightFrontMotorName("frontRightMotor")
		.rightRearMotorName("backRightMotor")
		.leftFrontMotorName("frontLeftMotor")
		.leftRearMotorName("backLeftMotor")
		.leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
		.leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
		.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
		.rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(52.085921878882594)
        .yVelocity(40.12158909623925);

	public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
		.forwardTicksToInches(9.092275468167541E-4)
		.strafeTicksToInches(9.040382814259659E-4)
		.turnTicksToInches(9.084995076982009E-4)
		.leftPodY(6.5)
		.rightPodY(-6.5)
		.strafePodX(-8.0)
		.leftEncoder_HardwareMapName("frontLeftMotor")
		.rightEncoder_HardwareMapName("backRightMotor")
		.strafeEncoder_HardwareMapName("backLeftMotor")
		.leftEncoderDirection(Encoder.REVERSE)
		.rightEncoderDirection(Encoder.REVERSE)
		.strafeEncoderDirection(Encoder.REVERSE)
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
