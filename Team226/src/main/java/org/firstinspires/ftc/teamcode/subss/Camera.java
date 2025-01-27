package org.firstinspires.ftc.teamcode.subss;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;




public class Camera {
    public OpenCvCamera controlHubCam;
    public Camera.ColorDetectionPipeline pipeline = new Camera.ColorDetectionPipeline();
    OpMode opmode;
    FtcDashboard dashboard;


    public Camera(OpMode _opMode){
            // Initialize the dashboard
            this.opmode = _opMode;
            dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier(
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

    public void init() {

    }

    public void color(){
        telemetry.addData("Status", "Streaming camera to telemetry");

        // Try to display the detected color from the pipeline
        try {
            telemetry.addData("Color", pipeline.returnColor());
            telemetry.addData("Distance ", pipeline.getDistance());
        } catch (NullPointerException e) {
            telemetry.addData("Color", "None"); // Ensures no exception during runtime
            telemetry.addData("Distance", "None");
        }

//        // Update telemetry with all relevant information
//        telemetry.update();
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
        public double distance;

        //@Override
        /*
        public Mat processFrame(Mat input) {//Color Detection
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


            telemetry.addData("Color Detected", colorDetected);
            telemetry.update();

            // Return the input frame (you could also return a processed frame if desired)
            return input;
        }*/

        public Mat processFrame(Mat input) {//Object Detection
            // Convert the input frame from BGR to HSV
            Mat hsvMat = new Mat();
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Create masks for red, yellow, and blue
            Mat redMask = new Mat();
            Mat yellowMask = new Mat();
            Mat blueMask = new Mat();

            Core.inRange(hsvMat, lowerRed, upperRed, redMask);
            Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);
            Core.inRange(hsvMat, lowerBlue, upperBlue, blueMask);

            // Process each color mask independently
            Rect redRect = processColorMask(redMask, new Scalar(255, 0, 0), input); // Red bounding box
            Rect yellowRect = processColorMask(yellowMask, new Scalar(0, 255, 255), input); // Yellow bounding box
            Rect blueRect = processColorMask(blueMask, new Scalar(0, 0, 255), input); // Blue bounding box

            // Detect color and calculate distance for the largest detected object
            if (redRect != null) {
                colorDetected = "Red";
                drawBoundingBox(input, redRect, new Scalar(255, 0, 0));
                distance = calculateDistance(redRect.height); // Draw red bounding box
            }

            if (yellowRect != null) {
                colorDetected = "Yellow";
                drawBoundingBox(input, yellowRect, new Scalar(0, 255, 255)); // Draw yellow bounding box
                distance = calculateDistance(yellowRect.height);
            }

            if (blueRect != null) {
                colorDetected = "Blue";
                drawBoundingBox(input, blueRect, new Scalar(0, 0, 255)); // Draw blue bounding box
                distance = calculateDistance(blueRect.height);
            }

            // Return the input frame with bounding boxes drawn on it
            return input;
        }


        private Rect processColorMask(Mat mask, Scalar color, Mat input) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                for (MatOfPoint contour : contours) {
                    if (Imgproc.contourArea(contour) > Imgproc.contourArea(largestContour)) {
                        largestContour = contour;
                    }
                }

                // Get bounding rectangle of the largest contour
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                Imgproc.rectangle(input, boundingRect, color, 2); // Draw the bounding box on the input frame
                return boundingRect;
            }

            return null; // No contour found
        }

        private void drawBoundingBox(Mat input, Rect rect, Scalar color) {
            Imgproc.rectangle(input, rect, color, 2);
        }

        private double calculateDistance(double objectHeightInPixels) {
            double KNOWN_OBJECT_HEIGHT = 89;
            double FOCAL_LENGTH = 4;
            return (KNOWN_OBJECT_HEIGHT * FOCAL_LENGTH) / objectHeightInPixels;
        }

        public String returnColor(){
            return colorDetected;
        }
        public double getDistance(){  return distance; }
    }


}