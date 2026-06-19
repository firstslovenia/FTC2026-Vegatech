
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - blue (near goal)", group = "Examples")
public class BlueMoveAutoNearGoal extends MoveAuto {
    public BlueMoveAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(24.3, 130, Math.toRadians(144));
        final Pose intermediate_pose = new Pose(start_pose.getX() + 30, start_pose.getY() - 40, start_pose.getHeading());
        final Pose end_pose = CommonPositions.BLUE_AUTO_END_POSE_NEAR_GOAL;

        startPose = start_pose;
        intermediatePose = intermediate_pose;
        endPose = end_pose;
        ballPickupXOffset = ballPickupXOffsetBlue;
    }
}
