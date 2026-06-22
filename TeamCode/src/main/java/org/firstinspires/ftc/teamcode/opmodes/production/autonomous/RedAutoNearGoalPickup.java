package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - red (near goal & pickup)")
public class RedAutoNearGoalPickup extends ShootingAutoWithPickup {
    public RedAutoNearGoalPickup() {
        super();

        startPose = CommonPositions.RED_AUTO_NEAR_GOAL_START_POSE;
        shootPose = CommonPositions.RED_AUTO_NEAR_GOAL_SHOOT_POSE;
        endPose = CommonPositions.RED_AUTO_END_POSE_NEAR_GOAL;
        goalPose = redGoalPose;
    }
}
