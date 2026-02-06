package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.generic.BallColor;
import org.firstinspires.ftc.teamcode.generic.Team;

@TeleOp(name = "Main (Pur, Pur, Gre)", group = "Production")
public class MainPreloadedBalls extends Main {
    @Override
    public void spindexer_override() {
        spindexer.balls = new BallColor[] { BallColor.Purple, BallColor.Purple, BallColor.Green };
    }
}
