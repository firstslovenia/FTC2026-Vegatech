package org.firstinspires.ftc.teamcode.opmodes.testing;

public enum HardwareComponent {
    frontLeftMotor,
    frontRightMotor,
    backLeftMotor,
    backRightMotor,
    odometry,
    shooterMotor,
    intakeMotor,
    spindexerMotor,
    Spindexer,
    shooterAngleServo,
    rgbLed,
    colorSensor,
    // Giblje med tko 0.3 in 1.0
    cameraAngleServo,
    lifterServo;

    private static final HardwareComponent[] values = values();

    public HardwareComponent next() {
        return values[(this.ordinal() + 1) % values.length];
    }

    public HardwareComponent previous() {
        return values[Math.floorMod(this.ordinal() - 1, values.length)];
    }
}