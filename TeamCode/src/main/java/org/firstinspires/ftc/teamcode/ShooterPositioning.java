package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterPositioning {

	/// How tilted up the camera is
	public static double CAMERA_ANGLE_DEGREES = 0.0;

	public static double APRILTAG_HEIGHT_CM = 16.5;

	/// How far away (in field coordinates) we want the target's x coordinate relative to the april tag
	public static double TARGET_TO_APRILTAG_X_DISTANCE_CM = 20.0;

	/// How far away (in field coordinates) we want the target's y coordinate relative to the april tag
	public static double TARGET_TO_APRILTAG_Y_DISTANCE_CM = 20.0;

	/// The angle of the apriltag to the field
	public static double ANGLE_OF_APRILTAG_RADS = Math.toRadians(36.0);

	public double x_distance_to_apriltag_m = 0.0;
	public double y_distance_to_apriltag_m = 0.0;
	public double x_distance_to_target_m = 0.0;
	public double y_distance_to_target_m = 0.0;

	/// Caluclates the target information we need to shoot at it;
	///
	/// angle_to_apriltag_rads is how much we need to turn to be facing directly
	/// at the apriltag (the apriltag pose yaw).
	public TargetInformation compute_target_information(double distance_to_apriltag_m, double angle_to_apriltag_rads) {
		TargetInformation info = new TargetInformation();

        // Compensation factor for the mounting
        //angle_to_apriltag_rads -= Math.toRadians(3.0);

		x_distance_to_apriltag_m = Math.cos(angle_to_apriltag_rads - ANGLE_OF_APRILTAG_RADS) * distance_to_apriltag_m;
		y_distance_to_apriltag_m = Math.sin(angle_to_apriltag_rads - ANGLE_OF_APRILTAG_RADS) * distance_to_apriltag_m;

		x_distance_to_target_m = x_distance_to_apriltag_m + TARGET_TO_APRILTAG_X_DISTANCE_CM / 100.0;
		y_distance_to_target_m = -y_distance_to_apriltag_m + TARGET_TO_APRILTAG_Y_DISTANCE_CM / 100.0;

		double distance_to_target_m = Math.sqrt(Math.pow(x_distance_to_target_m, 2) + Math.pow(y_distance_to_target_m, 2));

		double angle_to_target = Math.atan(x_distance_to_target_m / y_distance_to_target_m);

		// How much we need to turn to be at 0 degrees
		double angle_to_zero = angle_to_apriltag_rads + (Math.PI / 2.0 - ANGLE_OF_APRILTAG_RADS);
		double angle_diff_to_target = angle_to_zero - angle_to_target;

        info.tag_distance_m = distance_to_apriltag_m;
		info.distance_m = distance_to_target_m;

		info.angle_distance_to_zero = angle_to_zero;
		info.ideal_angle_to_target = angle_to_target;
		info.angle_distance_rads = angle_diff_to_target;

        info.time_ms = System.currentTimeMillis();

		return info;
	}
}
