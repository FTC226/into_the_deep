package org.firstinspires.ftc.teamcode.TestPrograms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "adaptive claw")
public class adaptiveClaw extends LinearOpMode {
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    SampleDetection pipeline = new SampleDetection();

    double cameraAngle = pipeline.returnAngle();

    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Angle", pipeline.returnAngle());
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class SampleDetection extends OpenCvPipeline {
        double angle;
        // Lower/Upper Color Bounds
        private final Scalar lowerRed1 = new Scalar(0, 150, 50);   // Red lower bound (hue wrap-around)
        private final Scalar upperRed1 = new Scalar(10, 255, 255); // Red upper bound
        private final Scalar lowerRed2 = new Scalar(170, 150, 50); // Second range of red (for wrap-around hue)
        private final Scalar upperRed2 = new Scalar(180, 255, 255);

        private final Scalar lowerBlue = new Scalar(100, 150, 50); // Blue lower bound
        private final Scalar upperBlue = new Scalar(130, 255, 255);// Blue upper bound

        private final Scalar lowerYellow = new Scalar(20, 150, 50); // Yellow lower bound
        private final Scalar upperYellow = new Scalar(30, 255, 255); // Yellow upper bound

        private Mat hsv = new Mat();
        private Mat maskRed = new Mat();
        private Mat maskBlue = new Mat();
        private Mat maskYellow = new Mat();
        private Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Red color masking (combine two masks for the hue wrap-around)
            Core.inRange(hsv, lowerRed1, upperRed1, maskRed);
            Mat maskRed2 = new Mat();
            Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
            Core.bitwise_or(maskRed, maskRed2, maskRed);

            // Blue color masking
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

            // Yellow color masking
            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);

            // Process red, blue, and yellow
            detectAndDrawRectangles(input, maskRed, new Scalar(255, 0, 0)); // Red bounding box
            detectAndDrawRectangles(input, maskBlue, new Scalar(0, 0, 255)); // Blue bounding box
            detectAndDrawRectangles(input, maskYellow, new Scalar(0, 255, 255)); // Yellow bounding box

            // Return the frame with rectangles and angles drawn
            return input;
        }

        /**
         * Detects the largest rectangle in the given mask, draws its bounding box on the frame, and displays its angle.
         * @param input The original frame.
         * @param mask The binary mask for a specific color.
         * @param boxColor The color for the bounding box (Scalar format: BGR).
         */

        @SuppressLint("DefaultLocale")
        private void detectAndDrawRectangles(Mat input, Mat mask, Scalar boxColor) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Find the largest contour
                double maxArea = 5000;
                MatOfPoint largestContour = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    // Find the minimum area bounding rectangle for the largest contour
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                    // Draw the rectangle
                    Point[] rectPoints = new Point[4];
                    rect.points(rectPoints);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(input, rectPoints[i], rectPoints[(i + 1) % 4], boxColor, 2);
                    }

                    // Calculate the angle
                    angle = rect.angle;
                    if (rect.size.width < rect.size.height) {
                        angle += 90;
                    }
                    if (angle < 0) {
                        angle += 180;
                    }

                    // Display the angle on the frame
                    Imgproc.putText(input, "Angle: " + String.format("%.2f", angle), rect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
                    telemetry.addData("Angle", angle);
                }
            }
        }
        public double returnAngle(){
            return angle;
        }
    }
    public double getAngle() {
        return cameraAngle;
    }
}
