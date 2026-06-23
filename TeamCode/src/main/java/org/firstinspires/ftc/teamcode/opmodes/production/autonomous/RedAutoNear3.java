package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Near Goal | 3")
public class RedAutoNear3 extends GenericAuto3 {
    public RedAutoNear3() {
        super();

        startPose = CommonPositions.RED_AUTO_NEAR_START;
        shootPose = CommonPositions.RED_AUTO_NEAR_SHOOT;
        endPose = CommonPositions.RED_AUTO_NEAR_END;
        goalPose = redGoalPose;
    }
}
