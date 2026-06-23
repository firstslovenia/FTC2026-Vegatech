
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue | Far Zone | 6")
public class BlueAutoFar6 extends GenericAuto6 {
    public BlueAutoFar6() {
        super();

        startPose = CommonPositions.BLUE_AUTO_FAR_START;
        shootPose = CommonPositions.BLUE_AUTO_FAR_SHOOT;
        pickupStartPose = CommonPositions.BLUE_AUTO_FAR_START_PICKUP;
        pickupEndPose = CommonPositions.BLUE_AUTO_FAR_END_PICKUP;
        endPose = CommonPositions.BLUE_AUTO_FAR_END;
        goalPose = blueGoalFarPose;
    }
}
