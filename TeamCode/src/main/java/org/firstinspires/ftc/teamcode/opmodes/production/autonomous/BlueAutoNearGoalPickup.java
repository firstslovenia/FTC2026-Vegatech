
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - blue (near goal & pickup)", group = "Examples")
public class BlueAutoNearGoalPickup extends ShootingAutoWithPickup {
    public BlueAutoNearGoalPickup() {
        super();

        startPose = CommonPositions.BLUE_AUTO_NEAR_GOAL_START_POSE;
        shootPose = CommonPositions.BLUE_AUTO_NEAR_GOAL_SHOOT_POSE;
        endPose = CommonPositions.BLUE_AUTO_END_POSE_NEAR_GOAL;
        goalPose = blueGoalPose;
    }
}
