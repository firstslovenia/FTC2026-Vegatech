package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - red (near goal)")
public class RedMoveAutoNearGoal extends MoveAuto {
    public RedMoveAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(119.6, 130, Math.toRadians(36));
        final Pose end_pose = new Pose(95.0, 32.0, Math.toRadians(0));

        startPose = start_pose;
        endPose = end_pose;
        goalPose = redGoalPose;
        ballPickupXOffset = ballPickupXOffsetRed;
    }
}
