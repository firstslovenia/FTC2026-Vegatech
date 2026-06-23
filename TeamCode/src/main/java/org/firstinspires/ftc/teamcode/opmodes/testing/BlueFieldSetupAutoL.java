
package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.GenericAuto0;

@Autonomous(name = "!! Setup Blue Goal L")
public class BlueFieldSetupAutoL extends GenericAuto0 {
    public BlueFieldSetupAutoL() {
        super();

        startPose = CommonPositions.BLUE_AUTO_NEAR_START;
        intermediatePose = new Pose(startPose.getX() + 18, startPose.getY() - 16, startPose.getHeading());
        endPose = new Pose(15.0, 110.0, Math.toRadians(90.0));
    }
}
