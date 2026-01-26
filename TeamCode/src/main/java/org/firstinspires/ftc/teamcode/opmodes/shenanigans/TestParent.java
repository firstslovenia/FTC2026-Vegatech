package org.firstinspires.ftc.teamcode.opmodes.shenanigans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.generic.Team;

@TeleOp(name = "Test Parent", group = "shenanigans")
public class TestParent extends LinearOpMode {

    LedIndicator ledIndicator;
    Hardware hardware;

    Team team = null;

    @Override
    public void runOpMode() {
        hardware = new Hardware(this);
        ledIndicator = new LedIndicator(this, hardware);

        waitForStart();

        while (true) {

            if (team == null) {
                ledIndicator.setPosition(LedIndicator.ORANGE_POSITION);
            } else{
                switch (team) {
                    case Red:
                        ledIndicator.setPosition(LedIndicator.RED_POSITION);
                        break;
                    case Blue:
                        ledIndicator.setPosition(LedIndicator.BLUE_POSITION);
                        break;
                }
            }

            sleep(1000);
        }
    }
}
