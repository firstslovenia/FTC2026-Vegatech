package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.generic.Team;

@TeleOp(name = "Main (Red)", group = "Production")
public class MainRed extends Main {
    public MainRed() {
        super();

        team = Team.Red;

        // Auto end pose
        starting_pos = ShooterPositioning.to_pose2d(new Pose(95.0, 32.0, Math.toRadians(0)));
    }
}
