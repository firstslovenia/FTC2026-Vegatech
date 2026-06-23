
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue | Near Goal | 0")
public class BlueAutoNear0 extends GenericAuto0 {
    public BlueAutoNear0() {
        super();

        startPose = CommonPositions.BLUE_AUTO_NEAR_START;
        intermediatePose = new Pose(startPose.getX() + 18, startPose.getY() - 16, startPose.getHeading());
        endPose = CommonPositions.BLUE_AUTO_NEAR_END;
    }
}
