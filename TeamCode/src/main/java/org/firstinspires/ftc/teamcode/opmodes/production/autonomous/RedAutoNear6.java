package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Near Goal | 6")
public class RedAutoNear6 extends GenericAuto6 {
    public RedAutoNear6() {
        super();

        startPose = CommonPositions.RED_AUTO_NEAR_START;
        shootPose = CommonPositions.RED_AUTO_NEAR_SHOOT;
        pickupStartPose = CommonPositions.RED_AUTO_NEAR_START_PICKUP;
        pickupEndPose = CommonPositions.RED_AUTO_NEAR_END_PICKUP;
        endPose = CommonPositions.RED_AUTO_NEAR_END;
        goalPose = redGoalPose;
    }
}
