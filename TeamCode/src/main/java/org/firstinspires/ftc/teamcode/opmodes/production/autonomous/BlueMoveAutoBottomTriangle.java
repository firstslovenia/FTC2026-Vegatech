
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - blue (triangle)")
public class BlueMoveAutoBottomTriangle extends MoveAuto {
    public BlueMoveAutoBottomTriangle() {
        super();

        final Pose start_pose = new Pose(56.0, 8.0, Math.toRadians(90));
        final Pose end_pose = new Pose(48.0, 32.0, Math.toRadians(180));

        startPose = start_pose;
        endPose = end_pose;
        goalPose = blueGoalFarPose;
        ballPickupXOffset = ballPickupXOffsetBlue;
    }
}
