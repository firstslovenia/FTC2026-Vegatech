package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - red (triangle)", group = "Examples")
public class RedAutoBottomTriangle extends ShootingAuto {
    public RedAutoBottomTriangle() {
        super();

        final Pose start_pose = new Pose(87.8, 8.5, Math.toRadians(90));
        final Pose shoot_pose = new Pose(87.5, 108, Math.toRadians(36));
        final Pose end_pose = new Pose(87.5, 60, Math.toRadians(0));

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
    }
}
