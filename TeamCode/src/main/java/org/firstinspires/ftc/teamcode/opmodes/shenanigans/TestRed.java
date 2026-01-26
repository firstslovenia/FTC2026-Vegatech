package org.firstinspires.ftc.teamcode.opmodes.shenanigans;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.generic.Team;

@TeleOp(name = "Test (Red)", group = "shenanigans")
public class TestRed extends TestParent {
    public TestRed() {
        super();

        team = Team.Red;
    }
}
