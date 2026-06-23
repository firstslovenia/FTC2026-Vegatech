package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.GenericAuto6;

@Autonomous(name = "Red | Far Zone | 6")
public class RedAutoFar6 extends GenericAuto6 {
    public RedAutoFar6() {
        super();

        startPose = CommonPositions.RED_AUTO_FAR_START;
        shootPose = CommonPositions.RED_AUTO_FAR_SHOOT;
        pickupStartPose = CommonPositions.RED_AUTO_FAR_START_PICKUP;
        pickupEndPose = CommonPositions.RED_AUTO_FAR_END_PICKUP;
        endPose = CommonPositions.RED_AUTO_FAR_END;
        goalPose = redGoalFarPose;
    }
}
