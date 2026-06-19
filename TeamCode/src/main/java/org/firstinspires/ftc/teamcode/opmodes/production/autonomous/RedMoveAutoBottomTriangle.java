package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - red (triangle)", group = "Examples")
public class RedMoveAutoBottomTriangle extends MoveAuto {
    public RedMoveAutoBottomTriangle() {
        super();

        final Pose start_pose = new Pose(88.0, 8.0, Math.toRadians(90));
        final Pose intermediate_pose = new Pose(start_pose.getX(), start_pose.getY() + 12, start_pose.getHeading());
        final Pose end_pose = CommonPositions.RED_AUTO_END_POSE;

        startPose = start_pose;
        intermediatePose = intermediate_pose;
        endPose = end_pose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
