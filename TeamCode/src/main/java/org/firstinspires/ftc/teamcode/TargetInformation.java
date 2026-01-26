package org.firstinspires.ftc.teamcode;

public class TargetInformation {
	/// How far we need to turn to be aligned with the target
	public double angle_distance_rads;

	/// How far we need to turn to be aligned with the playing field
	public double angle_distance_to_zero;

	/// What the ideal target angle (relative to the playing field) is
	public double ideal_angle_to_target;

	/// How far away we are from the target
	public double distance_m;

    ///  When this information was accurate
    public long time_ms = 0;

    public boolean partial_eq(TargetInformation other) {

        if (other == null) {
            return false;
        }

        return Math.abs(other.ideal_angle_to_target - ideal_angle_to_target) < (Math.PI / 180.0);
    }
}
