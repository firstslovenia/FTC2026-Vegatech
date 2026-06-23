
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue | Far Zone | 3")
public class BlueAutoFar3 extends GenericAuto3 {
    public BlueAutoFar3() {
        super();

        startPose = CommonPositions.BLUE_AUTO_FAR_ZONE_START_POSE;
        shootPose = CommonPositions.BLUE_AUTO_FAR_ZONE_SHOOT_POSE;
        endPose = CommonPositions.BLUE_AUTO_END_POSE;
        goalPose = blueGoalFarPose;
    }
}
