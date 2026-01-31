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

    /// Computes the smaller angle difference to the given position which puts the spidexer into the wanted angle
    public static double calculate_best_angle_diff_for(double angle_rads, double wanted_angle_rads) {
        angle_rads = to_positive(angle_rads);
        wanted_angle_rads = to_positive(wanted_angle_rads);

        double diff = wanted_angle_rads - angle_rads;
        double diff_other_way;

        if (diff < 0.0) {
            diff_other_way = diff + Math.PI * 2.0;
        } else {
            diff_other_way = diff - Math.PI * 2.0;
        }

        if (Math.abs(diff_other_way) < Math.abs(diff)) {
            return diff_other_way;
        } else {
            return diff;
        }
    }
}
