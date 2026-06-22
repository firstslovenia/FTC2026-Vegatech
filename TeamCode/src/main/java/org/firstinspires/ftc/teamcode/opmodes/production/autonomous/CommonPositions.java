package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;

public class CommonPositions {

    /// Start positions
    public static final Pose BLUE_AUTO_NEAR_GOAL_START_POSE = new Pose(24.3, 130, Math.toRadians(144));
    public static final Pose BLUE_AUTO_FAR_ZONE_START_POSE = new Pose(56.0, 8.0, Math.toRadians(90));
    public static final Pose RED_AUTO_NEAR_GOAL_START_POSE = new Pose(119.6, 130, Math.toRadians(36));
    public static final Pose RED_AUTO_FAR_ZONE_START_POSE = new Pose(88.0, 8.0, Math.toRadians(90));

    /// End positions
    public static final Pose BLUE_AUTO_END_POSE = new Pose(38.0, 12.0, Math.toRadians(180));
    public static final Pose BLUE_AUTO_END_POSE_NEAR_GOAL = new Pose(58.0, 114.0, Math.toRadians(180));
    public static final Pose RED_AUTO_END_POSE = new Pose(106.0, 12.0, Math.toRadians(0));
    public static final Pose RED_AUTO_END_POSE_NEAR_GOAL = new Pose(88.0, 114.0, Math.toRadians(0));

    /// Shooting positions
    public static final Pose BLUE_AUTO_NEAR_GOAL_SHOOT_POSE = new Pose(60.0, 95.0, Math.toRadians(0));
    public static final Pose BLUE_AUTO_FAR_ZONE_SHOOT_POSE = new Pose(60.0, 22.0, Math.toRadians(0));

    public static final Pose RED_AUTO_NEAR_GOAL_SHOOT_POSE = new Pose(85.0, 95.0, Math.toRadians(0));
    public static final Pose RED_AUTO_FAR_ZONE_SHOOT_POSE = new Pose(86.0, 20.0, Math.toRadians(0));

    /// Intake when near goal
    public static final Pose RED_START_PICKUP_NEAR_GOAL_POSE = new Pose(98.0, 83.0, Math.toRadians(0));
    public static final Pose RED_END_PICKUP_NEAR_GOAL_POSE = new Pose(124.0, 83.0, Math.toRadians(0));

    public static final Pose BLUE_START_PICKUP_NEAR_GOAL_POSE = new Pose(45.0, 83.0, Math.toRadians(180));
    public static final Pose BLUE_END_PICKUP_NEAR_GOAL_POSE = new Pose(19.0, 83.0, Math.toRadians(180));

    /// Intake when in far zone
    public static final Pose RED_START_PICKUP_FAR_ZONE_POSE = new Pose(98.0, 35.0, Math.toRadians(0));
    public static final Pose RED_END_PICKUP_FAR_ZONE_POSE = new Pose(124.0, 35.0, Math.toRadians(0));

    public static final Pose BLUE_START_PICKUP_FAR_ZONE_POSE = new Pose(45.0, 35.0, Math.toRadians(180));
    public static final Pose BLUE_END_PICKUP_FAR_ZONE_POSE = new Pose(19.0, 35.0, Math.toRadians(180));
}
