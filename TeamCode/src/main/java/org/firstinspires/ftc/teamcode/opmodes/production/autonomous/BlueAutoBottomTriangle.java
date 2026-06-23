
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue | Far Zone | 3")
public class BlueAutoBottomTriangle extends ShootingAuto {
    public BlueAutoBottomTriangle() {
        super();

        startPose = CommonPositions.BLUE_AUTO_FAR_ZONE_START_POSE;
        shootPose = CommonPositions.BLUE_AUTO_FAR_ZONE_SHOOT_POSE;
        endPose = CommonPositions.BLUE_AUTO_END_POSE;
        goalPose = blueGoalFarPose;
    }
}
