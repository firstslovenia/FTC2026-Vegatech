package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Far Zone | 0")
public class RedAutoFar0 extends GenericAuto0 {
    public RedAutoFar0() {
        super();

        startPose = CommonPositions.RED_AUTO_FAR_START;
        intermediatePose = new Pose(startPose.getX(), startPose.getY() + 12, startPose.getHeading());
        endPose = CommonPositions.RED_AUTO_FAR_END;
    }
}
