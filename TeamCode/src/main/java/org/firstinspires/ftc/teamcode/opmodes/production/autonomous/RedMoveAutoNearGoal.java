package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Near Goal | 0")
public class RedMoveAutoNearGoal extends MoveAuto {
    public RedMoveAutoNearGoal() {
        super();

        startPose = CommonPositions.RED_AUTO_NEAR_GOAL_START_POSE;
        intermediatePose = new Pose(startPose.getX() - 18, startPose.getY() - 16, startPose.getHeading());
        endPose = CommonPositions.RED_AUTO_END_POSE_NEAR_GOAL;
    }
}
