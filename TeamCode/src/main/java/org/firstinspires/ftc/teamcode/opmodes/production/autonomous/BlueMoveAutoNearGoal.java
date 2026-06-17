
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - blue (near goal)", group = "Examples")
public class BlueMoveAutoNearGoal extends MoveAuto {
    public BlueMoveAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(24.3, 130, Math.toRadians(144));
        final Pose end_pose = new Pose(48.0, 32.0, Math.toRadians(180));

        startPose = start_pose;
        endPose = end_pose;
        goalPose = blueGoalPose;
        ballPickupXOffset = ballPickupXOffsetBlue;
    }
}
