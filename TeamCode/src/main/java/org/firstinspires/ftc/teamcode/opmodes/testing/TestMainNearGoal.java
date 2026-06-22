package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.CommonPositions;
import org.firstinspires.ftc.teamcode.opmodes.production.autonomous.RedAutoNearGoal;

@TeleOp(name = "Testing Prod Odometry")
public class TestMainNearGoal extends Main {
    public TestMainNearGoal() {
        super();

        team = Team.Red;

        // Taken from red auto near goal
        starting_pos = ShooterPositioning.to_pose2d(new Pose(85.0, 95.0, Math.toRadians(36)));
    }
}
