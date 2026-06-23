
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue | Far Zone | 0")
public class BlueAutoFar0 extends GenericAuto0 {
    public BlueAutoFar0() {
        super();

        startPose = CommonPositions.BLUE_AUTO_FAR_ZONE_START_POSE;
        intermediatePose = new Pose(startPose.getX(), startPose.getY() + 12, startPose.getHeading());
        endPose = CommonPositions.BLUE_AUTO_END_POSE;
    }
}
