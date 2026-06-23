package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;

public class CommonPositions {

    /// Start positions
    public static final Pose BLUE_AUTO_NEAR_START = new Pose(24.3, 130, Math.toRadians(144));
    public static final Pose BLUE_AUTO_FAR_START = new Pose(56.0, 8.0, Math.toRadians(90));
    public static final Pose RED_AUTO_NEAR_START = new Pose(119.6, 130, Math.toRadians(36));
    public static final Pose RED_AUTO_FAR_START = new Pose(88.0, 8.0, Math.toRadians(90));

    /// Shooting positions
    public static final Pose BLUE_AUTO_NEAR_SHOOT = new Pose(60.0, 95.0, Math.toRadians(0));
    public static final Pose BLUE_AUTO_FAR_SHOOT = new Pose(60.0, 22.0, Math.toRadians(0));

    public static final Pose RED_AUTO_NEAR_SHOOT = new Pose(85.0, 95.0, Math.toRadians(0));
    public static final Pose RED_AUTO_FAR_SHOOT = new Pose(86.0, 20.0, Math.toRadians(0));

    /// Intake when near goal
    public static final Pose RED_AUTO_NEAR_START_PICKUP = new Pose(98.0 - 1, 83.0 + 1, Math.toRadians(0));
    public static final Pose RED_AUTO_NEAR_END_PICKUP = new Pose(124.0, 83.0 + 1, Math.toRadians(0));

    public static final Pose BLUE_AUTO_NEAR_START_PICKUP = new Pose(45.0 + 1, 83.0 + 1, Math.toRadians(180));
    public static final Pose BLUE_AUTO_NEAR_END_PICKUP = new Pose(19.0, 83.0 + 1, Math.toRadians(180));

    /// Intake when in far zone
    public static final Pose RED_AUTO_FAR_START_PICKUP = new Pose(98.0 - 1, 35.0 + 1, Math.toRadians(0));
    public static final Pose RED_AUTO_FAR_END_PICKUP = new Pose(124.0, 35.0 + 1, Math.toRadians(0));

    public static final Pose BLUE_AUTO_FAR_START_PICKUP = new Pose(45.0 + 1, 35.0 + 1, Math.toRadians(180));
    public static final Pose BLUE_AUTO_FAR_END_PICKUP = new Pose(19.0, 35.0 + 1, Math.toRadians(180));

    public static final Pose RED_AUTO_MIDDLE_START_PICKUP = new Pose(98.0 - 1, 60.0 + 1, Math.toRadians(0));
    public static final Pose BLUE_AUTO_MIDDLE_START_PICKUP = new Pose(45.0 + 1, 60.0 + 1, Math.toRadians(180));

    /// End positions
    public static final Pose BLUE_AUTO_FAR_END = new Pose(38.0, 12.0, Math.toRadians(180));
    public static final Pose BLUE_AUTO_NEAR_END = BLUE_AUTO_MIDDLE_START_PICKUP;
    public static final Pose RED_AUTO_FAR_END = new Pose(106.0, 12.0, Math.toRadians(0));
    public static final Pose RED_AUTO_NEAR_END = RED_AUTO_MIDDLE_START_PICKUP;
}
