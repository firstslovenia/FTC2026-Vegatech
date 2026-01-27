package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Webcam {

	LinearOpMode callingOpMode;
	Hardware hardware;

	public static final int TAG_ID_GREEN_PURPLE_PURPLE = 21;

	public static final int TAG_ID_PURPLE_GREEN_PURPLE = 22;
	public static final int TAG_ID_PURPLE_PURPLE_GREEN = 23;
	public static final int TAG_ID_BLUE = 20;
	public static final int TAG_ID_RED = 24;

	public ColorOrder order = ColorOrder.Unknown;

    public TargetInformation target_position;
    public ShooterPositioning shooter_positioning = new ShooterPositioning();

	public ArrayList<AprilTagDetection> last_detections = new ArrayList<>();

	/**
	 * The variable to store our instance of the AprilTag processor.
	 */
	private AprilTagProcessor aprilTag;

	/**
	 * The variable to store our instance of the vision portal.
	 */
	private VisionPortal visionPortal;

	public Webcam(LinearOpMode callingOpMode, Hardware hardware) {
		aprilTag = new AprilTagProcessor.Builder()

		// The following default settings are available to un-comment and edit as needed.
		//.setDrawAxes(false)
		//.setDrawCubeProjection(false)
		//.setDrawTagOutline(true)
		//.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
		//.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
		//.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
			.setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
			.setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())

		// == CAMERA CALIBRATION ==
		// If you do not manually specify calibration parameters, the SDK will attempt
		// to load a predefined calibration for your camera.
		//.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
		// ... these parameters are fx, fy, cx, cy.
                // 1280 x 720 for C270
                .setLensIntrinsics(1420.13799378, 1420.13799378, 676.976315018, 340.62607304)
		.build();

		// Adjust Image Decimation to trade-off detection-range for detection-rate.
		// eg: Some typical detection data using a Logitech C920 WebCam
		// Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
		// Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
		// Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
		// Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
		// Note: Decimation can be changed on-the-fly to adapt during a match.
		//aprilTag.setDecimation(3);

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();
		builder.setCamera(hardware.webcam);

		// Choose a camera resolution. Not all cameras support all resolutions.
		//builder.setCameraResolution(new Size(640, 480));
		builder.setCameraResolution(new Size(1280, 720));
        //builder.setCameraResolution(new Size(960, 720));
		//builder.setCameraResolution(new Size(1920, 1080));

		// Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
		//builder.enableLiveView(true);

		// Set the stream format; MJPEG uses less bandwidth than default YUY2.
		//builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
		builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

		// Choose whether or not LiveView stops if no processors are enabled.
		// If set "true", monitor shows solid orange screen if no processors enabled.
		// If set "false", monitor shows camera view without annotations.
		//builder.setAutoStopLiveView(false);

		// Set and enable the processor.
		builder.addProcessor(aprilTag);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();

		// Disable or re-enable the aprilTag processor at any time.
		//visionPortal.setProcessorEnabled(aprilTag, true);
	}

	/// Returns the april tags the camera saw in the last loop.
	public ArrayList<AprilTagDetection> getApriltags() {
		return last_detections;
	}

	public void update() {
		last_detections = aprilTag.getDetections();

		for (int i = 0; i < last_detections.size(); i++) {
			AprilTagDetection detection = last_detections.get(i);

			switch (detection.id) {
				case TAG_ID_GREEN_PURPLE_PURPLE:
					order = ColorOrder.GreenPurplePurple;
					break;
				case TAG_ID_PURPLE_GREEN_PURPLE:
					order = ColorOrder.PurpleGreenPurple;
					break;
				case TAG_ID_PURPLE_PURPLE_GREEN:
					order = ColorOrder.PurplePurpleGreen;
					break;
                case TAG_ID_BLUE:
                case TAG_ID_RED:

                    double distance_to_apriltag_m = detection.ftcPose.range;

                    // Mystical magical conversion factor, actually measured in the workshop.
                    //
                    // Don't ask me how we got here. This works.
                    // You wanna know where you got this? The resolution was wrong... by a factor of 0.75x!!
                    //distance_to_apriltag_m *= 0.75;

                    TargetInformation target_pos = shooter_positioning.compute_target_information(distance_to_apriltag_m, detection.ftcPose.yaw);

                    // Check that it is not complete garbage
                    if (shooter_positioning.x_distance_to_target_m < 0.05 ||
                            shooter_positioning.y_distance_to_target_m < 0.05 ||
                            target_pos.distance_m < 0.05 ||
                            Math.abs(target_pos.angle_distance_rads) > Math.PI / 2.0) {
                        continue;
                    }

                    target_position = target_pos;
                    break;
			}
		}
	}
}
