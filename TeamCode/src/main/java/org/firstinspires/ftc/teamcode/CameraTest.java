package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@TeleOp(name = "CameraTest")
public class CameraTest extends OpMode {

    private OpenCvCamera controlHubCam;
    private ColorDetectionPipeline pipeline = new ColorDetectionPipeline();// Assign the pipeline object to a variable name
    // Use OpenCvCamera class from FTC SDK

    public void init(){
        // Initialize the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Get camera ID from the robot configuration
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Initialize the webcam
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Start streaming the camera to FtcDashboard

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(controlHubCam, 30);

                // Set the pipeline for color detection
                controlHubCam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened");
            }
        });



    }

    public void loop(){
        telemetry.addData("Status", "Streaming camera to telemetry");

        try{
            telemetry.addData("Color", pipeline.returnColor());
        } catch (NullPointerException e){
            telemetry.addData("Color", "None");//ensures there is no exception during run time in the program
        }

        //telemetry.addData("Color Detected", colorDetected);
        telemetry.update();
    }

    class ColorDetectionPipeline extends OpenCvPipeline {

        // Define HSV ranges for colors
        Scalar lowerRed = new Scalar(0, 100, 100);   // Lower bound for red
        Scalar upperRed = new Scalar(10, 255, 255);  // Upper bound for red

        Scalar lowerYellow = new Scalar(20, 100, 100);   // Lower bound for yellow
        Scalar upperYellow = new Scalar(30, 255, 255);  // Upper bound for yellow

        Scalar lowerBlue = new Scalar(110, 100, 100);   // Lower bound for blue
        Scalar upperBlue = new Scalar(130, 255, 255);   // Upper bound for blue
        public String colorDetected;

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame from BGR to HSV
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Create masks for each color
            Mat redMask = new Mat();
            Mat yellowMask = new Mat();
            Mat blueMask = new Mat();

            Core.inRange(hsvMat, lowerRed, upperRed, redMask);
            Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);
            Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);

            // Count non-zero pixels in each mask
            double redCount = Core.countNonZero(redMask);
            double yellowCount = Core.countNonZero(yellowMask);
            double blueCount = Core.countNonZero(blueMask);

            // Determine which color has the most pixels
            colorDetected = "None";
            if (redCount > yellowCount && redCount > blueCount) {
                colorDetected = "Red";
            } else if (yellowCount > redCount && yellowCount > blueCount) {
                colorDetected = "Yellow";
            } else if (blueCount > redCount && blueCount > yellowCount) {
                colorDetected = "Blue";
            } else {
                colorDetected = "None";
            }

            // Send color data to telemetry
            /*
            telemetry.addData("Red", redCount);
            telemetry.addData("Blue", blueCount);
            telemetry.addData("Yellow", yellowCount);*/

            telemetry.addData("Color Detected", colorDetected);
            telemetry.update();

            // Return the input frame (you could also return a processed frame if desired)
            return input;
        }

        public String returnColor(){
            return colorDetected;
        }
    }
}