package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - red (triangle)", group = "Examples")
public class RedAutoBottomTriangle extends ShootingAuto {
    public RedAutoBottomTriangle() {
        super();

        final Pose start_pose = new Pose(88.0, 8.0, Math.toRadians(90));
        final Pose shoot_pose = new Pose(86.0, 20.0, Math.toRadians(0));
        final Pose end_pose = new Pose(95.0, 32.0, Math.toRadians(0));

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
        goalPose = redGoalFarPose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
