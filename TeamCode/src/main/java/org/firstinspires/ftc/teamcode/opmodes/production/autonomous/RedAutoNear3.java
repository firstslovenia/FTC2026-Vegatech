package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Near Goal | 3")
public class RedAutoNear3 extends GenericAuto3 {
    public RedAutoNear3() {
        super();

        startPose = CommonPositions.RED_AUTO_NEAR_GOAL_START_POSE;
        shootPose = CommonPositions.RED_AUTO_NEAR_GOAL_SHOOT_POSE;
        endPose = CommonPositions.RED_AUTO_END_POSE_NEAR_GOAL;
        goalPose = redGoalPose;
    }
}
