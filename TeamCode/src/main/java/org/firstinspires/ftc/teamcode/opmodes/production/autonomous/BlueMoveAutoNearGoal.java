
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - blue (near goal)", group = "Examples")
public class BlueMoveAutoNearGoal extends MoveAuto {
    public BlueMoveAutoNearGoal() {
        super();

        startPose = CommonPositions.BLUE_AUTO_NEAR_GOAL_START_POSE;
        intermediatePose = new Pose(startPose.getX() + 18, startPose.getY() - 16, startPose.getHeading());
        endPose = CommonPositions.BLUE_AUTO_END_POSE_NEAR_GOAL;
    }
}
