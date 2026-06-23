package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Far Zone | 3")
public class RedAutoBottomTriangle extends ShootingAuto {
    public RedAutoBottomTriangle() {
        super();

        startPose = CommonPositions.RED_AUTO_FAR_ZONE_START_POSE;
        shootPose = CommonPositions.RED_AUTO_FAR_ZONE_SHOOT_POSE;
        endPose = CommonPositions.RED_AUTO_END_POSE;
        goalPose = redGoalFarPose;
    }
}
