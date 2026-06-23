package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red | Far Zone | 0", group = "Examples")
public class RedMoveAutoBottomTriangle extends MoveAuto {
    public RedMoveAutoBottomTriangle() {
        super();

        startPose = CommonPositions.RED_AUTO_FAR_ZONE_START_POSE;
        intermediatePose = new Pose(startPose.getX(), startPose.getY() + 12, startPose.getHeading());
        endPose = CommonPositions.RED_AUTO_END_POSE;
    }
}
