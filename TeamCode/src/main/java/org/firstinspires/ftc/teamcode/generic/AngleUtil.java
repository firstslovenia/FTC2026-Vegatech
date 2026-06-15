package org.firstinspires.ftc.teamcode.generic;

public class AngleUtil {

    /// Makes a positive angle from a potentially negative one
    public static double to_positive(double angle_rads) {
        while (angle_rads < 0.0) {
            angle_rads += Math.PI * 2.0;
        }

        while (angle_rads > Math.PI * 2.0) {
            angle_rads -= Math.PI * 2.0;
        }

        return angle_rads;
    }

    /// Computes the negative angle difference to the given position which puts the spidexer into the wanted angle
    public static double ensure_negative_diff(double angle_rads, double wanted_angle_rads) {
        angle_rads = to_positive(angle_rads);
        wanted_angle_rads = to_positive(wanted_angle_rads);

        double diff = wanted_angle_rads - angle_rads;

        if (diff > 0.0) {
            diff -= Math.PI * 2.0;
        }

        return diff;
    }

    /// Computes the positive angle difference to the given position which puts the spidexer into the wanted angle
    public static double ensure_positive_diff(double angle_rads, double wanted_angle_rads) {
        angle_rads = to_positive(angle_rads);
        wanted_angle_rads = to_positive(wanted_angle_rads);

        double diff = wanted_angle_rads - angle_rads;

        if (diff < 0.0) {
            diff += Math.PI * 2.0;
        }

        return diff;
    }
}
