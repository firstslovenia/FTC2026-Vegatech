
package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.ShootingAuto;

@Autonomous(name = "Testing Auto - shoot w/o movement")
public class SimpleTestShootingAuto extends ShootingAuto {
    public SimpleTestShootingAuto() {
        super();

        final Pose start_pose = new Pose(75.0, 90.0, Math.toRadians(90));
        final Pose shoot_pose = start_pose.withY(start_pose.getY() + 1.0);
        final Pose end_pose = start_pose;

        startPose = start_pose;
        shootPose = shoot_pose;
        endPose = end_pose;
        goalPose = start_pose.withY(shoot_pose.getY() + 60.0);

        // Do not actually turn
        obeliskPose = goalPose.withX(goalPose.getX() + 20.0);
    }
}
