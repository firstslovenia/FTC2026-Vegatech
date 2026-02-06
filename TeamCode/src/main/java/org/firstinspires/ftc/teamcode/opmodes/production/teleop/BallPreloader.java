package org.firstinspires.ftc.teamcode.opmodes.production.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Spindexer;
import org.firstinspires.ftc.teamcode.generic.BallColor;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;
import org.firstinspires.ftc.teamcode.generic.Team;
import org.firstinspires.ftc.teamcode.generic.Vector2D;

@TeleOp(name = "Ball Preloading", group = "Production")
public class BallPreloader extends LinearOpMode {

    Hardware hardware;
    LedIndicator ledIndicator;
    Spindexer spindexer;

    boolean intake_enabled = false;

    @Override
    public void runOpMode() {

        hardware = new Hardware(this);
        hardware.init();

        ledIndicator = new LedIndicator(this, hardware);
        ledIndicator.setPosition(LedIndicator.OFF_POSITION);

        spindexer = new Spindexer(this, hardware);

        waitForStart();
        spindexer.init();
        spindexer.switch_to_holding_pattern();

        while (opModeIsActive()) {

            // If spindexer is on, update LED
            if (spindexer.ball_to_intake != null) {
                ledIndicator.setPosition(LedIndicator.VIOLET_POSITION);
            } else {
                ledIndicator.setPosition(LedIndicator.OFF_POSITION);
            }

            // Enable / disable intake
            if (gamepad1.xWasPressed()) {
                intake_enabled = !intake_enabled;

                if (intake_enabled) {
                    hardware.intakeMotor.setPower(1.0);
                } else {
                    hardware.intakeMotor.setPower(0.0);
                }
            }

            // User 2

            // Do spindexer intake
            if (gamepad1.yWasPressed()) {
                if (spindexer.ball_to_intake != null) {
                    spindexer.switch_to_holding_pattern();
                } else {
                    spindexer.switch_to_available_intake();
                }
            }

            // Get ready for match position
            if (gamepad1.aWasPressed()) {
                spindexer.move_to_angle(0.0);
            }

            spindexer.update();

            telemetry.update();
        }
    }
}