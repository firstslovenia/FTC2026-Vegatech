
package org.firstinspires.ftc.teamcode.opmodes.production.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Move auto - blue (triangle)")
public class BlueMoveAutoBottomTriangle extends MoveAuto {
    public BlueMoveAutoBottomTriangle() {
        super();

        startPose = CommonPositions.BLUE_AUTO_FAR_ZONE_START_POSE;
        intermediatePose = new Pose(startPose.getX(), startPose.getY() + 12, startPose.getHeading());
        endPose = CommonPositions.BLUE_AUTO_END_POSE;
    }
}
