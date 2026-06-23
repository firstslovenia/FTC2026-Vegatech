
package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.GenericAuto0;

@Autonomous(name = "!! Setup Blue Goal L")
public class BlueFieldSetupAutoL extends GenericAuto0 {
    public BlueFieldSetupAutoL() {
        super();

        Constants.driveConstants.setMaxPower(0.3);

        startPose = new Pose(15.0, 110.0, Math.toRadians(90.0));
        endPose = new Pose(28.0, 133.0, Math.toRadians(144.0));
        intermediatePose = new Pose(endPose.getX() + 18, endPose.getY() - 16, startPose.getHeading());
    }
}
