package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.ShooterPlusPlus;
import org.firstinspires.ftc.teamcode.ShooterPositioning;
import org.firstinspires.ftc.teamcode.TargetInformation;
import org.firstinspires.ftc.teamcode.generic.LedIndicator;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Webcam;
import org.firstinspires.ftc.teamcode.generic.SlidingWindow;

@TeleOp(name = "Kateri motorji so kje?", group = "Testing")
public class KateriMotorjiSoKje extends LinearOpMode {

    @Override
    public void runOpMode() {

        // 1 FL, +1 je nazaj
        // 2 BL, +1 je nazaj
        // 3 BR, +1 je naprej
        // 4 FR, +1 je naprej
        DcMotor[] motorji = { hardwareMap.get(DcMotor.class, "1"),  hardwareMap.get(DcMotor.class, "2"),  hardwareMap.get(DcMotor.class, "3"),  hardwareMap.get(DcMotor.class, "4") };
        int i = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                i = (i + 1) % motorji.length;
            } else if (gamepad1.bWasPressed()) {
                i = (i - 1) % motorji.length;

                if (i < 0) {
                    i += motorji.length;
                }
            }

            DcMotor motor = motorji[i];

            float power = -gamepad1.right_stick_y;
            motor.setPower(power);

            telemetry.addData("Motor", motor.getDeviceName());
            telemetry.addData("i", i);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
