
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - blue (near goal)", group = "Examples")
public class BlueAutoNearGoal extends ShootingAuto {
    public BlueAutoNearGoal() {
        super();

        final Pose start_pose = new Pose(24.3, 130, Math.toRadians(144));
        final Pose shoot_pose = new Pose(56.6, 106, Math.toRadians(144));
        final Pose end_pose = new Pose(56.6, 60, Math.toRadians(180));

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
    }
}
