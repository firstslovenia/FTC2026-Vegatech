package org.firstinspires.ftc.teamcode.generic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;

public class LedIndicator {

	LinearOpMode callingOpMode;
	Hardware hardware;

	public static double RED_POSITION = 0.277;

	public static double ORANGE_POSITION = 0.333;
	public static double YELLOW_POSITION = 0.388;
	public static double LIGHT_GREEN_POSITION = 0.444;
	public static double GREEN_POSITION = 0.5;
	public static double AQUA_POSITION = 0.555;
	public static double BLUE_POSITION = 0.611;
	public static double INDIGO_POSITION = 0.666;
	public static double VIOLET_POSITION = 0.722;
	public static double OFF_POSITION = 0.0;
	public static double WHITE_POSITION = 1.0;

	public LedIndicator(LinearOpMode opMode, Hardware hw_map) {
		callingOpMode = opMode;
		hardware = hw_map;
		hardware.rgbLed.setPosition(0.0);
	}

	/// Maps the desired hue (degree) value to the servo position
	///
	/// The max hue (degrees) are 268; anything else is clamped
	public double hueToServoPosition(double hue) {
		hue = Math.max(Math.min(hue, 268.0), 0.0);

		return (hue / 268.0) * (VIOLET_POSITION - RED_POSITION) + RED_POSITION;
	}

	/// Sets the LED to the specified position (between 0 and 1)
	public void setPosition(double pos) {
		hardware.rgbLed.setPosition(pos);
	}

	/// Sets the led to the specified hue (in degrees)
	public void setHue(double hue) {
		setPosition(hueToServoPosition(hue));
	}

	/// Sets the led to (vaguely) the rgb color (0 - 1)
	///
	/// Only sets the hue, ignores the saturation and brightness
	public void setRGB(double r, double g, double b) {
		setPosition(hueToServoPosition(rgbToHue(r, g, b)));
	}

	/// Converts and rgb value (0 - 1) to hue (in degrees)
	///
	/// See https://stackoverflow.com/questions/39118528/rgb-to-hsl-conversion
	public double rgbToHue(double r, double g, double b) {
		double max = Math.max(Math.max(r, g), Math.max(g, b));
		double min = Math.min(Math.min(r, g), Math.min(g, b));
		double c = max - min;

		double hue;

		if (c == 0.0) {
			hue = 0.0;
		} else {
			if (max == r) {
				double segment = (g - b) / c;
				double shift = 0.0;

				if (segment < 0.0) {
					shift = 360.0 / 60.0;
				}

				hue = segment + shift;
			} else if (max == g) {
				double segment = (b - r) / c;
				double shift = 120.0 / 60.0;
				hue = segment + shift;
			} else {
				double segment = (r - g) / c;
				double shift = 240.0 / 60.0;
				hue = segment + shift;
			}
		}

		return hue * 60.0;
	}
}
