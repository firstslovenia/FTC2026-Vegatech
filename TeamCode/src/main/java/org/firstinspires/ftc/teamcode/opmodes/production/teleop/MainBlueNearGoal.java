package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;

@TeleOp(name = "Main | Blue | Near Goal", group = "Production")
public class MainBlueNearGoal extends Main {
    public MainBlueNearGoal() {
        super();

        team = Team.Blue;
        starting_pos = ShooterPositioning.to_pose2d(CommonPositions.BLUE_AUTO_NEAR_END);
    }
}
