package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;

@TeleOp(name = "Main | Red | Near Goal", group = "Production")
public class MainRedNearGoal extends Main {
    public MainRedNearGoal() {
        super();

        team = Team.Red;
        starting_pos = ShooterPositioning.to_pose2d(CommonPositions.RED_AUTO_NEAR_END);
    }
}
