package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - red (triangle)", group = "Examples")
public class RedMoveAutoBottomTriangle extends MoveAuto {
    public RedMoveAutoBottomTriangle() {
        super();

        final Pose start_pose = new Pose(88.0, 8.0, Math.toRadians(90));
        final Pose end_pose = new Pose(95.0, 32.0, Math.toRadians(0));

        startPose = start_pose;
        endPose = end_pose;
        goalPose = redGoalFarPose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
