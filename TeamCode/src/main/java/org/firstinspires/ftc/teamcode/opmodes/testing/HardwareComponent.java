package org.firstinspires.ftc.teamcode.opmodes.testing;

public enum HardwareComponent {
    frontLeftMotor,
    frontRightMotor,
    backLeftMotor,
    backRightMotor,
    leftForwardDeadwheel,
    rightForwardDeadwheel,
    backSidewaysDeadwheel,
    shooterMotor,
    shooterPusherServo,
    intakeMotor,
    spindexerMotor,
    Spindexer,
    rgbLed,
    colorSensor;

    private static final HardwareComponent[] values = values();

    public HardwareComponent next() {
        return values[(this.ordinal() + 1) % values.length];
    }

    public HardwareComponent previous() {
        return values[Math.floorMod(this.ordinal() - 1, values.length)];
    }
}