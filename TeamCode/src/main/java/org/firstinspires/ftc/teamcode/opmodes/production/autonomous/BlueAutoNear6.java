
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue | Near Goal | 6")
public class BlueAutoNear6 extends GenericAuto6 {
    public BlueAutoNear6() {
        super();

        startPose = CommonPositions.BLUE_AUTO_NEAR_START;
        shootPose = CommonPositions.BLUE_AUTO_NEAR_SHOOT;
        pickupStartPose = CommonPositions.BLUE_AUTO_NEAR_START_PICKUP;
        pickupEndPose = CommonPositions.BLUE_AUTO_NEAR_END_PICKUP;
        endPose = CommonPositions.BLUE_AUTO_NEAR_END;
        goalPose = blueGoalPose;
    }
}
