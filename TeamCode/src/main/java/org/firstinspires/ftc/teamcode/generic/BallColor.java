package org.firstinspires.ftc.teamcode.generic;

public enum BallColor {
    Purple,
    Green,
    ///  No ball is loaded into the spindexer currently
    None;

    public BallColor other() {
        switch (this) {
            case Purple:
                return Green;
            case Green:
                return Purple;
            default:
                return None;
        }
    }
}
