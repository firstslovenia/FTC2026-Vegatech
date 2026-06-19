package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - red (near goal)")
public class RedMoveAutoNearGoal extends MoveAuto {
    public RedMoveAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(119.6, 130, Math.toRadians(36));
        final Pose intermediate_pose = new Pose(start_pose.getX() - 18, start_pose.getY() - 16, start_pose.getHeading());
        final Pose end_pose = CommonPositions.RED_AUTO_END_POSE_NEAR_GOAL;

        startPose = start_pose;
        intermediatePose = intermediate_pose;
        endPose = end_pose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
