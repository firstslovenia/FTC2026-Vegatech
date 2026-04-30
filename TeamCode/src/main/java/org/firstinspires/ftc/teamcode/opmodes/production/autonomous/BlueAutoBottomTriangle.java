
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - blue (triangle)")
public class BlueAutoBottomTriangle extends ShootingAuto {
    public BlueAutoBottomTriangle() {
        super();

        final Pose start_pose = new Pose(56.0, 8.0, Math.toRadians(90));
        final Pose shoot_pose = new Pose(60.0, 22.0, Math.toRadians(0));
        final Pose end_pose = new Pose(48.0, 32.0, Math.toRadians(180));

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
        goalPose = blueGoalFarPose;
        ballPickupXOffset = ballPickupXOffsetBlue;
    }
}
