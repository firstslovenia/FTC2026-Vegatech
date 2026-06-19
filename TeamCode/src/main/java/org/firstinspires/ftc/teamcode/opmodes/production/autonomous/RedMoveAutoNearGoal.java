package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - red (near goal)")
public class RedMoveAutoNearGoal extends MoveAuto {
    public RedMoveAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(119.6, 130, Math.toRadians(36));
        final Pose intermediate_pose = new Pose(start_pose.getX() - 30, start_pose.getY() - 40, start_pose.getHeading());
        final Pose end_pose = CommonPositions.RED_AUTO_END_POSE;

        startPose = start_pose;
        intermediatePose = intermediate_pose;
        endPose = end_pose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
