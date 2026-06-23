
package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.GenericAuto0;

@Autonomous(name = "!! Setup Blue Goal Auto R")
public class BlueFieldSetupAutoR extends GenericAuto0 {
    public BlueFieldSetupAutoR() {
        super();

        startPose = CommonPositions.BLUE_AUTO_NEAR_GOAL_START_POSE;
        intermediatePose = new Pose(startPose.getX() + 18, startPose.getY() - 16, startPose.getHeading());
        endPose = new Pose(34.0, 136.0, Math.toRadians(90.0));
    }
}
