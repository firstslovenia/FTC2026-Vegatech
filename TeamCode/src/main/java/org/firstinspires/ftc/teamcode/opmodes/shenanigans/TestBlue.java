package org.firstinspires.ftc.teamcode.opmodes.shenanigans;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.generic.Team;

@TeleOp(name = "Test (Blue)", group = "shenanigans")
public class TestBlue extends TestParent {
    public TestBlue() {
        super();

        team = Team.Blue;
    }
}
