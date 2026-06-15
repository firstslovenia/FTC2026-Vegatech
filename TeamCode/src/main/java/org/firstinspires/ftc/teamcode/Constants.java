package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
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
        // Note: the defaults here will mess up everything you've already configured! Pedro is too smart
		.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
		.rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(52.085921878882594)
        .yVelocity(40.12158909623925);

	public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.3)
            .strafePodX(6.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odometry")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

	public static Follower createFollower(HardwareMap hardwareMap) {
		return new FollowerBuilder(followerConstants, hardwareMap)
			.pathConstraints(pathConstraints)
			.mecanumDrivetrain(driveConstants)
			.pinpointLocalizer(localizerConstants)
			.build();
	}
}
