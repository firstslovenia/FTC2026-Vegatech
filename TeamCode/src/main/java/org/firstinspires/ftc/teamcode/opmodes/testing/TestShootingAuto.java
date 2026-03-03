
package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.ShootingAuto;

@Autonomous(name = "Testing Auto - Shooting stuff")
public class TestShootingAuto extends ShootingAuto {
    public TestShootingAuto() {
        super();

        final Pose start_pose = new Pose(75.0, 90.0, Math.toRadians(90));
        final Pose shoot_pose = new Pose(90.0, 100.0, Math.toRadians(0.0));
        final Pose end_pose = new Pose(75.0, 90.0, Math.toRadians(0));

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
        goalPose = redGoalPose;
    }
}
