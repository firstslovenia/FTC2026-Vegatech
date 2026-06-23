package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.RedAutoNearGoal;

@TeleOp(name = "Main | Blue", group = "Production")
public class MainBlue extends Main {
    public MainBlue() {
        super();

        team = Team.Blue;
        starting_pos = ShooterPositioning.to_pose2d(CommonPositions.BLUE_AUTO_END_POSE);
    }
}
