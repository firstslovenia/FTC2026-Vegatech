package org.firstinspires.ftc.teamcode.opmodes.production.teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.generic.Team;

@TeleOp(name = "Main (Red)", group = "Production")
public class MainRed extends Main {
    public MainRed() {
        super();

        team = Team.Red;
    }
}
