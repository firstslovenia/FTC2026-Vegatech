package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.generic.Team;

public class ShooterPositioning {

	/// How tilted up the camera is
	public static double APRILTAG_HEIGHT_CM = 16.5;

	/// How far away (in field coordinates) we want the target's x coordinate relative to the april tag
	public static double TARGET_TO_APRILTAG_X_DISTANCE_CM = 8.0;

	/// How far away (in field coordinates) we want the target's y coordinate relative to the april tag
	public static double TARGET_TO_APRILTAG_Y_DISTANCE_CM = 6.0;

	/// The angle of the apriltag to the field
	public static double ANGLE_OF_APRILTAG_RADS = Math.toRadians(36.0);
    public static Pose2D BLUE_APRILTAG_POSE = new Pose2D(DistanceUnit.INCH, 7.0, 144.0, AngleUnit.DEGREES, -36.0);
    public static Pose2D RED_PARILTAG_POSE = new Pose2D(DistanceUnit.INCH, 138.0, 144.0, AngleUnit.DEGREES, -180 + 36.0);

    // Helper conversion multipliers
    public static double IN_TO_CM = 2.54;
    public static double IN_TO_M = IN_TO_CM / 100.0;
    public static double CM_TO_IN = 0.393701;
    public static double M_TO_IN = CM_TO_IN * 100.0;

	public double x_distance_to_apriltag_m = Double.NaN;
	public double y_distance_to_apriltag_m = Double.NaN;
	public double x_distance_to_target_in = Double.NaN;
	public double y_distance_to_target_in = Double.NaN;

    public static Pose BLUE_GOAL_TELEOP_POSE = new Pose(12.0, 135.0);
    public static Pose RED_GOAL_TELEOP_POSE = new Pose(132.0, 135.0);

    /// Converts a pedro pathing pose to a pose2d
    public static Pose2D to_pose2d(Pose pedro_pos) {
       return new Pose2D(DistanceUnit.INCH, pedro_pos.getX(), pedro_pos.getY(), AngleUnit.RADIANS, pedro_pos.getHeading());
    }

    /// Returns if two positions are roughly the same
    public static boolean pose2d_rougly_eq(Pose2D a, Pose2D b) {
        return Math.abs(a.getX(DistanceUnit.METER) - b.getX(DistanceUnit.METER)) < 0.2 && Math.abs(a.getY(DistanceUnit.METER) - b.getY(DistanceUnit.METER)) < 0.2 && Math.abs(a.getHeading(AngleUnit.DEGREES) - b.getHeading(AngleUnit.DEGREES)) < 10;
    }

    /// Calculates where we are from the camera detection
    public Pose2D compute_current_pos_from_camera(double distance_to_apriltag_m, double yaw_rads, double bearing_rads, Team goal_team) {

        Pose2D goal_pose = goal_team == Team.Blue ? BLUE_APRILTAG_POSE : RED_PARILTAG_POSE;

        // How much we need to turn to be at 0 degrees
        double angle_to_zero = Math.PI - yaw_rads + goal_pose.getHeading(AngleUnit.RADIANS);

        // The angle of the right triangle (x distance to apriltag), (y distance to apriltag) (distance to apriltag)
        double angle_of_triangle_x_y_dist = angle_to_zero - bearing_rads;
        x_distance_to_apriltag_m = Math.cos(angle_of_triangle_x_y_dist) * distance_to_apriltag_m;
        y_distance_to_apriltag_m = Math.sin(angle_of_triangle_x_y_dist) * distance_to_apriltag_m;

        return new Pose2D(DistanceUnit.METER, goal_pose.getX(DistanceUnit.METER) - x_distance_to_apriltag_m, goal_pose.getY(DistanceUnit.METER) - y_distance_to_apriltag_m, AngleUnit.RADIANS, angle_to_zero);
    }

	/// Calculates the target information we need to shoot;
	///
	/// yaw_rads is how much we need to turn to be facing directly
	/// at the apriltag (the apriltag pose yaw).
	public TargetInformation compute_target_information_from_camera(double distance_to_apriltag_m, double yaw_rads, double bearing_rads) {
		TargetInformation info = new TargetInformation();

        // Compensation factor for the mounting
        //yaw_rads -= Math.toRadians(3.0);

        // How much we need to turn to be at 0 degrees
        double angle_to_zero = yaw_rads + ANGLE_OF_APRILTAG_RADS;

        // The angle of the right triangle (x distance to apriltag), (y distance to apriltag) (distance to apriltag)
        double angle_of_triangle_x_y_dist = angle_to_zero - bearing_rads;
		x_distance_to_apriltag_m = Math.cos(angle_of_triangle_x_y_dist) * distance_to_apriltag_m;
		y_distance_to_apriltag_m = Math.sin(angle_of_triangle_x_y_dist) * distance_to_apriltag_m;

		x_distance_to_target_in = x_distance_to_apriltag_m + TARGET_TO_APRILTAG_X_DISTANCE_CM / 100.0;
		y_distance_to_target_in = y_distance_to_apriltag_m + TARGET_TO_APRILTAG_Y_DISTANCE_CM / 100.0;

		double distance_to_target_m = Math.sqrt(Math.pow(x_distance_to_target_in, 2) + Math.pow(y_distance_to_target_in, 2));

        // Positive angle from the x axis
		double angle_to_target = Math.atan(y_distance_to_target_in / x_distance_to_target_in);

		double angle_diff_to_target = angle_to_zero - angle_to_target;

        info.tag_distance_m = distance_to_apriltag_m;
		info.distance_m = distance_to_target_m;

		info.angle_distance_to_zero = angle_to_zero;
		info.ideal_angle_to_target = angle_to_target;
		info.angle_distance_rads = angle_diff_to_target;

        info.time_ms = System.currentTimeMillis();

		return info;
	}

    /// Calculates the target information we need to shoot, given just our current pos and team
    public TargetInformation compute_target_information(Pose2D pos, Team team) {
        Pose goal_pos = team == Team.Blue ? BLUE_GOAL_TELEOP_POSE : RED_GOAL_TELEOP_POSE;

        return compute_target_information_for_two_pos(pos, ShooterPositioning.to_pose2d(goal_pos));
    }

    /// Calculates the target information we need to shoot given our current pos and a target
    public TargetInformation compute_target_information_for_two_pos(Pose2D pos, Pose2D target_pos) {
        TargetInformation info = new TargetInformation();

        double angle_to_zero = -pos.getHeading(AngleUnit.RADIANS);

        x_distance_to_target_in = target_pos.getX(DistanceUnit.INCH) - pos.getX(DistanceUnit.INCH);
        y_distance_to_target_in = target_pos.getY(DistanceUnit.INCH) - pos.getY(DistanceUnit.INCH);

        double distance_to_target_in = Math.sqrt(Math.pow(x_distance_to_target_in, 2) + Math.pow(y_distance_to_target_in, 2));

        // Positive angle from the x axis
        double angle_to_target = Math.atan(y_distance_to_target_in / x_distance_to_target_in);

        info.y = y_distance_to_target_in;
        info.x = x_distance_to_target_in;

        if (x_distance_to_target_in < 0.0) {
            angle_to_target = Math.PI + angle_to_target;
        }

        double angle_diff_to_target = angle_to_zero + angle_to_target;

        info.tag_distance_m = Double.NaN;
        info.distance_m = distance_to_target_in * IN_TO_M;

        info.angle_distance_to_zero = angle_to_zero;
        info.ideal_angle_to_target = angle_to_target;
        info.angle_distance_rads = angle_diff_to_target;

        info.time_ms = System.currentTimeMillis();

        return info;
    }
}
