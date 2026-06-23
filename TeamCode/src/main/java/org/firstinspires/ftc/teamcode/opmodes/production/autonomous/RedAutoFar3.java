package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Far Zone | 3")
public class RedAutoFar3 extends GenericAuto3 {
    public RedAutoFar3() {
        super();

        startPose = CommonPositions.RED_AUTO_FAR_START;
        shootPose = CommonPositions.RED_AUTO_FAR_SHOOT;
        endPose = CommonPositions.RED_AUTO_FAR_END;
        goalPose = redGoalFarPose;
    }
}
