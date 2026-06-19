package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - red (near goal)")
public class RedAutoNearGoal extends ShootingAuto {
    public RedAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(119.6, 130, Math.toRadians(36));
        final Pose shoot_pose = new Pose(85.0, 95.0, Math.toRadians(0));
        final Pose end_pose = CommonPositions.RED_AUTO_END_POSE_NEAR_GOAL;

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
        goalPose = redGoalPose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
