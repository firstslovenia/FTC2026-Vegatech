package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.generic.BallColor;

public enum ColorOrder {
	Unknown,
	GreenPurplePurple,
	PurpleGreenPurple,
	PurplePurpleGreen;

    public BallColor[] as_array() {
        switch (this) {
            case GreenPurplePurple:
                return new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple};
            case PurpleGreenPurple:
                return new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
            case PurplePurpleGreen:
            default:
                return new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green};
        }
    }

    /// Returns the correct color for the nth ball we want to shoot
    public BallColor color_for_ith(int i) {
        return as_array()[i % 3];
    }
}