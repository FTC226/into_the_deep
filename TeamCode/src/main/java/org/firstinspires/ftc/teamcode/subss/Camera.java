package org.firstinspires.ftc.teamcode.subss;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subss.vison.SampleDetector;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvPipeline;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;



import org.apache.commons.math3.linear.*;


public class Camera extends OpenCvPipeline {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public double distance;
    public double realArea;

    public double realXMatrix;
    public double realYMatrix;

    ArrayList<RotatedRect> filteredRects;
    double minDistanceThreshold = 1000;

    OpMode opMode;

    public Mat frame;
    private Mat hsv;
    private Mat gray;
    private Mat inRange;
    private Mat kernel;
    private Mat kernel2;
    private Mat hierarchy;
    private Mat mask;

    private MatOfDouble muMat;
    private MatOfDouble sigmaMat;

    private Mat inHRange;
    private Mat inSVRange;

//    double realerX;
//    double realerY;
//    double bigAngle;

    boolean angle =false;

    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED();
    }



    public static Telemetry telemetry;

    public static Scalar lowerYellow = new Scalar(19.0, 150.0, 130.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    //public static Scalar lowerBlue = new Scalar(90.0, 150.0, 100.0); // hsv
    public static Scalar lowerBlue = new Scalar(50.0, 50.0, 80.0); // hsv
    public static Scalar upperBlue = new Scalar(120.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 150.0, 120.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private double sampleAngle = 0;
    private double k_translation = 1d/640d;

    public static SampleColor colorType = SampleColor.BLUE;
    //@Override
    public void init(/*int width, int height, CameraCalibration calibration*/) {
        frame = new Mat();
        hsv = new Mat();
        gray = new Mat();
        mask = new Mat();
        inRange = new Mat();

        muMat = new MatOfDouble();
        sigmaMat = new MatOfDouble();

        inHRange = new Mat();
        inSVRange = new Mat();

        kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(25, 25));
        kernel2 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(10, 10));

        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        hierarchy = new Mat();


        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();

        List<MatOfPoint> filteredContours = new ArrayList<>();


        ArrayList<ArrayList<Double[]>> overlapGroups = new ArrayList<>();

        ArrayList<Point> real = new ArrayList<>();


        MatOfPoint2f intersection = new MatOfPoint2f();


        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();


        MatOfPoint points = new MatOfPoint();


        List<MatOfPoint> listThing = new ArrayList<>();
    }
    /*

        public SampleDetector(Telemetry telemetry){
            this.telemetry = telemetry;
    //        this.colorType = colorType;
        }

     */
    public Camera(OpMode _opMode) {
        opMode = _opMode;
        telemetry = _opMode.telemetry;
    }

    //@Override
    public Mat processFrame(Mat input) {
        frame = input.clone();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);


        // Getting representative brightness of image and correcting brightness
        Core.meanStdDev(gray, muMat, sigmaMat);
//        telemetry.addData("gray mu", muMat.get(0,0)[0]);
//        telemetry.addData("gray sigma", sigmaMat.get(0,0)[0]);

        double mu = muMat.get(0,0)[0];
        double sigma = sigmaMat.get(0,0)[0];
        double k = 1;
        Scalar lowerBound = new Scalar(mu-k*sigma);
        Scalar upperBound = new Scalar(mu+k*sigma);

        Core.inRange(gray, lowerBound, upperBound, mask);
        Scalar maskedMean = Core.mean(gray, mask);
        double averageInRange = maskedMean.val[0];
        double targetAverageInRange = 90;
        frame.convertTo(frame, -1, targetAverageInRange/averageInRange, 0);

        telemetry.addData("averageInRange", averageInRange);

        // Color threshold
//        Core.inRange(hsv, PixelColor.YELLOW.LOWER, PixelColor.YELLOW.UPPER, inRange);
        if (colorType.equals(SampleColor.BLUE)) {
            Core.inRange(hsv, lowerBlue, upperBlue, inRange);
        } else if (colorType.equals(SampleColor.RED)) {
            Core.inRange(hsv, lowerRedH, upperRedH, inHRange);
            Core.bitwise_not(inHRange, inHRange);
            Core.inRange(hsv, lowerRedSV, upperRedSV, inSVRange);
            Core.bitwise_and(inHRange, inSVRange, inRange);
        } else {
            Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        }

//        Imgproc.erode(inRange, inRange, kernel);
//        Imgproc.dilate(inRange, inRange, kernel2);

        // Find all contours
//        Imgproc.erode(inRange, inRange, kernel);
//        Imgproc.dilate(inRange, inRange, kernel2);

        // Find all contours
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by size and get rotated rects
        int minArea = 2000;
        int largest = 10000;
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        List<MatOfPoint> filteredContours = new ArrayList<>();
        Iterator<MatOfPoint> iterator = unfilteredContours.iterator();
        while (iterator.hasNext()) {
            MatOfPoint contour = iterator.next();
            RotatedRect minAreaRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double area = minAreaRect.size.area();
            if (area <= minArea || area >=largest) {
                iterator.remove(); // Safely remove the element
            } else {
                filteredContours.add(contour);
                rotatedRects.add(minAreaRect);
            }
        }
        Imgproc.drawContours(frame, filteredContours, -1, new Scalar(0, 255, 0), 1);

        // Get overlapping rotated rect groups
        double overlapThreshold = 0.3; // % of smaller box covered
        Set<Integer> toSkip = new HashSet<>();
        ArrayList<ArrayList<Double[]>> overlapGroups = new ArrayList<>();
        for (int i = 0; i < rotatedRects.size(); i++) {
            if (toSkip.contains(i)) continue;
            toSkip.add(i);
            ArrayList<Double[]> overlapGroup = new ArrayList<>();
            double iArea = rotatedRects.get(i).size.area();
            overlapGroup.add(new Double[]{(double)i, iArea});
            for (int j = i+1; j < rotatedRects.size(); j++) {
                if (toSkip.contains(j)) continue;
                double jArea = rotatedRects.get(j).size.area();
                double overlapArea = getIntersectionArea(rotatedRects.get(i), rotatedRects.get(j));
                double distance = Math.sqrt(
                        Math.pow(rotatedRects.get(i).center.x - rotatedRects.get(j).center.x, 2) +
                                Math.pow(rotatedRects.get(i).center.y - rotatedRects.get(j).center.y, 2)
                );
                // Only merge if overlap is significant AND rectangles are close together
                if (overlapArea / Math.min(iArea, jArea) >= overlapThreshold && distance <= minDistanceThreshold) {
                    overlapGroup.add(new Double[]{(double)j, jArea});
                    toSkip.add(j);
                }
            }
            overlapGroups.add(overlapGroup);
        }

        // telemetry
        ArrayList<ArrayList<Double>> overlapGroups2 = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            overlapGroups2.add(new ArrayList<>());
            for (Double[] index : overlapGroup) {
                overlapGroups2.get(overlapGroups2.size()-1).add(index[0]);
            }
        }
        telemetry.addData("overlapGroups", overlapGroups2);

        // Filter out overlapping rotated rects
        filteredRects = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            int maxIndex = overlapGroup.get(0)[0].intValue();
            double maxArea = overlapGroup.get(0)[1];
            for (Double[] rect : overlapGroup) {
                if (rect[1] > maxArea) {
                    maxArea = rect[1];
                    maxIndex = rect[0].intValue();
                }
            }
            filteredRects.add(rotatedRects.get(maxIndex));
        }

        telemetry.addData("filteredRects.size()", filteredRects.size());


//        // Draw unfiltered rects as blue
//        for (RotatedRect rotatedRect : rotatedRects) {
//            Point[] vertices = new Point[4];
//            rotatedRect.points(vertices);
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 0, 255), 2);
//            }
//        }

        // Draw filtered rects as green
        ArrayList<Point> real = getOffsets(filteredRects);
        for (int i = 0; i < filteredRects.size(); i++) {
            RotatedRect rotatedRect = filteredRects.get(i);
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Point center = rotatedRect.center;



            double procAngle = rotatedRect.angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;

            double length = 200;
            double vecX = center.x+length*Math.cos(Math.toRadians(procAngle));
            double vecY = center.y-length*Math.sin(Math.toRadians(procAngle));
//            telemetry.addData("vecX", vecX);
//            telemetry.addData("vecY", vecY);
            if (vecX < 0) {
                vecY = center.y - (1+vecX/(length*Math.cos(Math.toRadians(procAngle))))*length*Math.sin(Math.toRadians(procAngle));
                vecX = 0;
            } if (vecY < 0) {
                vecX = center.x + (1+vecY/(length*Math.sin(Math.toRadians(procAngle))))*length*Math.cos(Math.toRadians(procAngle));
                vecY = 0;
            } if (vecX > 640) {
                vecY = center.y - (1-(vecX-640)/ (length*Math.cos(Math.toRadians(procAngle))))*length*Math.sin(Math.toRadians(procAngle));
                vecX = 640;
            } if (vecY > 480) {
                vecX = center.x + (1+(vecY-480)/(length*Math.sin(Math.toRadians(procAngle))))*length*Math.cos(Math.toRadians(procAngle));
                vecY = 480;
            }
            Imgproc.line(frame, center, new Point(vecX,vecY), new Scalar(0, 255, 255), 1);
            Imgproc.line(frame, center, new Point(center.x+length/2,center.y), new Scalar(0, 255, 255), 1);
            // Define arc parameters
            int radius = 20;
            double endAngle = -procAngle;
            Scalar color = new Scalar(0, 255, 255);
            int thickness = 1;

            // Generate points on the arc
            MatOfPoint points = new MatOfPoint();
            Imgproc.ellipse2Poly(center, new Size(radius, radius), 0, 0, (int) endAngle, 1, points);

            // Draw the arc
            List<MatOfPoint> listThing = new ArrayList<>();
            listThing.add(points);
            Imgproc.polylines(frame, listThing, false, color, thickness);
            Imgproc.putText(frame, (Math.round(10*procAngle)/10d)+" deg", new Point(center.x+30, center.y-10), 0, 0.5, new Scalar(0, 255, 255));

            double sampleHeight = (1d/rotatedRect.size.area()+2.28e-5)/(7.14e-6);
            sampleHeight = (1/rotatedRect.size.area()+2.57e-5)/(7.6e-6); // calculate height of camer based on area of sample
            telemetry.addData("sampleHeight", sampleHeight);

            distance = calculateDistance(sampleHeight);

            double real_x = realX(); // in inches
            double real_y = realY();
            telemetry.addData("real_x", real_x);
            packet.put("real_x", real_x);
            packet.put("real_y", real_y);
            packet.put("both","{"+real_x+", " + real_y + ", }");
            double[] outputData = MatrixTransformation();
            realXMatrix = outputData[0];
//            realYMatrix = 4.57461*real_y + 12.95584;
            realYMatrix = (0.0786771*Math.pow(real_y, 4))
                    + (0.146952*Math.pow(real_y, 3))
                    + (0.79093*Math.pow(real_y, 2))
                    + (3.65908*Math.pow(real_y, 1))
                    + 12.32045;
            packet.put("Estimated Inches: X=", realXMatrix);
            packet.put("Estimated Inches: Y=", realYMatrix);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("real_y", real_y);
            Imgproc.line(frame, center, new Point(320, center.y), new Scalar(255, 255, 0), 1);
            Imgproc.putText(frame, (Math.round(10*real_x)/10d)+(Math.abs(real_x)>1?" in":""), new Point(320+(center.x-320)*0.5-20, center.y+15), 0, 0.5, new Scalar(255, 255, 0));
            Imgproc.line(frame, new Point(320, center.y), new Point(320,240), new Scalar(255, 255, 0), 1);
            Imgproc.putText(frame, (Math.round(10*real_y)/10d)+(Math.abs(real_y)>1?" in":""), new Point(315-10*Double.toString(Math.round(10*real_y)/10d).length()-(Math.abs(real_y)>1?22:0), 240+(center.y-240)*0.5+10), 0, 0.5, new Scalar(255, 255, 0));


            Imgproc.circle(frame, rotatedRect.center, 1, new Scalar(255, 255, 0), 3);
        }
        Imgproc.circle(frame, new Point(320, 240), 1, new Scalar(255, 255, 0), 3);



        // telemetry
        if (!filteredRects.isEmpty()) {
//            telemetry.addData("width ", filteredRects.get(0).size.width);
//            telemetry.addData("height ", filteredRects.get(0).size.height);
            telemetry.addData("area ", filteredRects.get(0).size.area());

//            telemetry.addData("angle ", filteredRects.get(0).angle);
//            telemetry.addData("center ", filteredRects.get(0).center);
//            telemetry.addData("center scaled", new Point((filteredRects.get(0).center.x - 320) / 640 * 3.0/8.0, -(filteredRects.get(0).center.y - 240) / 480));
//            output.add(new Point((i.center.x - 320) / 640 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));
            double procAngle = filteredRects.get(0).angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
//            telemetry.addData("procAngle ", procAngle);
            sampleAngle = procAngle;
        }
//        telemetry.addData("sampleAngle", sampleAngle);
//        telemetry.addData("Distance", distance);







        telemetry.update();


        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }

    private double getIntersectionArea(RotatedRect rect1, RotatedRect rect2) {
        // Get vertices of the rectangles
        Point[] vertices1 = new Point[4];
        rect1.points(vertices1);

        Point[] vertices2 = new Point[4];
        rect2.points(vertices2);

        // Convert vertices arrays to MatOfPoint2f
        MatOfPoint2f poly1 = new MatOfPoint2f(vertices1);
        MatOfPoint2f poly2 = new MatOfPoint2f(vertices2);

        // Output MatOfPoint2f for the intersection polygon
        MatOfPoint2f intersection = new MatOfPoint2f();

        // Calculate intersection
        return Imgproc.intersectConvexConvex(poly1, poly2, intersection, true);
    }


    public MatOfPoint2f convertMatToMatOfPoint2f(Mat mat) {
        // Check if the Mat is in the correct format (CV_32FC2)
        if (mat.type() != CvType.CV_32FC2) {
            throw new IllegalArgumentException("Mat must be of type CV_32FC2");
        }

        // Create a MatOfPoint2f object
        MatOfPoint2f matOfPoint2f = new MatOfPoint2f();

        // Convert Mat rows to Point objects
        Point[] points = new Point[(int) mat.total()];
        for (int i = 0; i < mat.rows(); i++) {
            float[] data = new float[2];
            mat.get(i, 0, data);
            points[i] = new Point(data[0], data[1]);
        }

        // Set points to MatOfPoint2f
        matOfPoint2f.fromArray(points);

        return matOfPoint2f;
    }

    public ArrayList<Point> getOffsets(ArrayList<RotatedRect> input) {
        // Note: This method only works when the camera is directly above the samples, looking straight down

        ArrayList<Point> output = new ArrayList<Point>();
        double cameraAngle = 0;

        double height = 10.0; // in inches
        // TODO: Make height not hardcoded, instead base it off of robot position


        double canvasVertical = height*3.0/8.0; // inches
        double canvasHorizontal = height / 2.0;

        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            output.add(new Point((i.center.x - 320) / 320 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }


        return output;
    }

    public double realX() {
        if (filteredRects.isEmpty()) return 0; // No rectangles detected
        ArrayList<Point> real = getOffsets(filteredRects);

        // Find the rectangle with the lowest realY value
        int minYIndex = 0;
        double minY = real.get(0).y;
        for (int i = 1; i < real.size(); i++) {
            if (real.get(i).y < minY) {
                realArea = filteredRects.get(i).size.area();
                minY = real.get(i).y;
                minYIndex = i;
            }
        }

        // Ensure minYIndex is within bounds
        if (minYIndex >= 0 && minYIndex < filteredRects.size() && minYIndex < real.size()) {
            // Draw a bright red contour around the chosen rectangle
            RotatedRect chosenRect = filteredRects.get(minYIndex);
            Point[] vertices = new Point[4];
            chosenRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 0, 0), 4); // Bright red contour
            }

            // Return the realX of the rectangle with the lowest realY
            return real.get(minYIndex).x;
        } else {
            telemetry.addData("Error", "Invalid minYIndex: " + minYIndex);
            return 0;
        }
    }

    public double getRealArea() {
        return realArea;
    }

    public double realY() {
        if (filteredRects.isEmpty()) return 0; // No rectangles detected
        ArrayList<Point> real = getOffsets(filteredRects);

        // Find the rectangle with the lowest realY value
        int minYIndex = 0;
        double minY = real.get(0).y;
        for (int i = 1; i < real.size(); i++) {
            if (real.get(i).y < minY) {
                minY = real.get(i).y;
                minYIndex = i;
            }
        }

        // Return the realY of the rectangle with the lowest realY
        return real.get(minYIndex).y;
    }

    public double realAngle() {
        if (filteredRects.isEmpty()) return 0; // No rectangles detected
        ArrayList<Point> real = getOffsets(filteredRects);

        // Find the rectangle with the lowest realY value
        int minYIndex = 0;
        double minY = real.get(0).y;
        for (int i = 1; i < real.size(); i++) {
            if (real.get(i).y < minY) {
                minY = real.get(i).y;
                minYIndex = i;
            }
        }

        // Get the angle of the rectangle with the lowest realY
        RotatedRect targetRect = filteredRects.get(minYIndex);
        double angle = targetRect.angle;

        // Adjust the angle based on rectangle orientation
        if (targetRect.size.width > targetRect.size.height) {
            angle *= -1;
        } else {
            angle = 90 - angle;
        }

        return angle;
    }

    public double getRealXMatrix() {
        return realXMatrix;
    }

    public double getRealYMatrix() {
        return realYMatrix;
    }

    private double calculateDistance(double objectHeightInPixels) {
        double KNOWN_OBJECT_HEIGHT = 89;
        double FOCAL_LENGTH = 4;
        return (KNOWN_OBJECT_HEIGHT * FOCAL_LENGTH) / objectHeightInPixels;
    }

    public double getDistance(){  return distance; }


    public void setColor(String color){
        if(color.equals("blue")){
            colorType = SampleColor.BLUE;
        } else if(color.equals("red")){
            colorType = SampleColor.RED;
        } else if(color.equals("yellow")){
            colorType = SampleColor.YELLOW;
        }
    }

    public double[] MatrixTransformation() {
        double[][] inputData = {
                {-0.6562495231628418, 0.4140622615814209, 2.0, 0},
                {1.4065628051757812, 1.6775000095367432, -5.5, 0},
                {1.6590147018432617, 0.4481973648071289, -4.5, 0},
                {-2.6480417251586914, -0.07192468643188477, 6.0, 0},
                {0.13501930236816406, 1.375683069229126, -0.1, 0},
                {1.5557489395141602, 0.7365269660949707, -3.5, 0},
                {2.329928398132324, -0.3894233703613281, -4.5, 0},
                {-3.7540390491485596, -0.8121838569641113, 7, 0},
                {-2.280069351196289, 0.6705715656280518, 6.5, 0},
                {1.674018383026123, 1.0803275108337402, -4.1, 0},
                {-2.7781505584716797, -0.8471565246582031, 5.25, 0},
                {2.1376609802246094, -0.812842845916748, -3.8, 0},
                {3.9182558059692383, 0.3498799800872803, -9.12, 0},
                {-3.843922972679138, 0.5793275833129883, 9.65, 0},
                {-0.7470364570617676, -0.9730601310729979, 1.6, 0},
                {1.411851406097412, 0.6558308601379395, -3.5, 0},
                {3.252963066101074, 0.6681859493255615, -6.9, 0},
                {1.8828125, -1.1953125, -2.75, 0},
                {-1.9765625, -0.0546875, 4.6, 0},
                {1.9214386940002441, -1.3307805061340332, -3.12, 0},
                {2.1171875, 0.7265625, -5.1, 0},
                {-1.5580658912658691, 0.8312921524047852, 4.8, 0},
                {-1.1903128623962402, 1.4881248474121094, 4.75, 0},
                {1.946850299835205, 1.2122178077697754, -5.36, 0},
                {-1.3671875, -0.0859375, 3.125, 0},
                {-1.1348590850830078, 0.55476975440979, 3.125, 0},
                {-1.4296875, 1.078125, 4.5, 0},
                {-1.498551845550537, 1.4312236309051514, 5.25, 0},
                {-1.546954870223999, -0.4732394218444825, 3, 0},
                {1.7448434829711914, -0.2526741027832031, -3.6, 0},
                {1.278602123260498, 0.9172871112823486, -3.75, 0},
                {0.9651298522949219, 1.5611662864685059, -3.5, 0},
                {-1.552565097808838, -0.040128469467163086, 3.5, 0},
                {-2.745312213897705, 0.45937466621398926, 7, 0},
                {2.484495162963867, 1.1150240898132324, -7.35, 0},
                {2.8495335578918457, 0.16408872604370117, -6.75, 0},
                {1.128281593322754, -0.40192270278930664, -2.25, 0},
                {-1.1693480014801025, 0.6112194061279297, 3.25, 0},
                {-2.784764289855957, 0.3787224292755127, 7.25, 0},
                {2.4296875, 1.3671875, -7.75, 0},
                {2.7425684928894043, 0.7856326103210449, -7.5, 0},
                {3.2653121948242188, 0.025624752044677734, -7.25, 0},
                {3.1000289916992188, -0.6841230392456055, -5.75, 0},
                {2.6894636154174805, 1.3120081424713135, -8.6, 0},
                {-4.157591998577118, 1.3625452518463135, 13.75, 0},
                {-4.178485631942749, 1.482572078704834, 15, 0},
                {4.298190116882324, 1.632608413696289, -15.75, 0},
                {2.5859375, 1.609375, -9.25, 0},
                {-2.6328125, 1.4609375, 9, 0},
                {0, 1.4431838989257812, 0, 0},
                {-4.136507630348206, 0.9504857063293457, 12.8, 0},
                {-2.4765625, 0.9609374999999999, 7.5, 0},
                {0, 1, 0, 0},
                {2.2734375, 1.0625, -6.9, 0},
                {4.231176376342773, 1.1092703342437744, -13.4, 0},
                {-3.9765623807907104, -0.0312502384185791, 9.25, 0},
                {-1.9140625, -0.1484375, 4.4, 0},
                {0, 0, 0, 0},
                {1.9656133651733398, 0.03010940551757813, -4.35, 0},
                {4.062877655029297, 0.0706026554107666, -9.45, 0},
                {4.021059036254883, -0.8780989646911621, -7.25, 0},
                {2.0390625, -1.125, -3.25, 0},
                {0, -0.5, 0, 0},
                {-2.194890022277832, -1.1431589126586914, 4, 0},
                {-4.002005219459534, -1.2824254035949707, 7.5, 0},
                {-0.0390625, 0.3203125, 0, 0},
                {-0.2103252410888672, -0.9752326011657715, 0, 0},
                {1.317807674407959, 0.8307027816772461, -4, 0},
                {-1.9927971363067627, -0.9976372718811034, 3, 0},
                {1.4453125, -1.1171875, -2.5, 0},
                {-1.8921871185302734, 0.05156230926513672, 3.5, 0},
                {2.3984384536743164, -0.1328125, -5.2, 0},
                {3.9409027099609375, 0.2987372875213623, -8.75, 0},
                {-3.7372846603393555, 0.10414862632751465, 7.5, 0},
                {1.7310285568237305, -1.066746711730957, -3, 0},
                {2.363205909729004, -1.1385345458984375, -4, 0},
                {-1.1796875, 0.84375, 2.6, 0},
                {-1.1612482070922852, 1.5396208763122559, 3.25, 0},
                {1.3203125, 1.578125, -5, 0},
                {3.3806228637695312, 0.21774816513061523, -7.5, 0},
                {-4.362312614917755, -0.3315725326538086, 8.25, 0},
                {-0.7006969451904297, -0.4190244674682617, 1.5, 0},
                {-1.9219818115234375, 1.1278893947601318, 5.5, 0},
                {-1.469177007675171, 0.03353476524353027, 3.25, 0},
                {-0.9260101318359375, 1.868106484413147, 3, 0},
                {-2.1484375, 1.8359375, 7.5, 0},
                {-1.3258273601531982, 1.8501839637756348, 4.25, 0},
                {-1.079732894897461, 0.45059728622436523, 2.5, 0},
                {-0.14583683013916016, 1.6331121921539307, 0, 0},
                {-1.9827125072479248, 1.0646915435791016, 5.5, 0},
                {-1.8984375, 0.1640625, 3.55, 0},
                {-3.0098060369491577, 0.9135775566101074, 7.55, 0},
                {1.1734380722045898, 0.4656248092651367, -3.25, 0},
                {0.7747602462768555, 1.5050477981567383, -3.15, 0},
                {-2.1708431243896484, -0.8596696853637695, 3.5, 0}

        };

        // Compute transformation matrix
        // Compute homography matrix
        double[] coeffsX = computeHomography(inputData, true);
        double[] coeffsY = computeHomography(inputData, false);

        // Test: Convert new (realX, realY) from (cameraX, cameraY)
        return new double[]{transform(realX(), realY(), coeffsX), transform(realX(), realY(), coeffsY)};
    }

    public double[] computeHomography(double[][] points, boolean isX) {
        int n = points.length;
        if (n < 3) throw new IllegalArgumentException("Need at least 3 points for a 3-parameter homography");

        RealMatrix A = new Array2DRowRealMatrix(n, 6); // 3 columns for h0, h1, h2
        RealVector b = new ArrayRealVector(n);

        for (int i = 0; i < n; i++) {
            double x = points[i][0], y = points[i][1];  // Camera space
            double xReal = isX? points[i][2] : points[i][3];  // Real-world x-coordinate

            // Set up the equation for x'
            A.setRow(i, new double[]{1, x, y, x*x, y*y, x*y});
            b.setEntry(i, xReal);
        }

        // Solve for the homography parameters
        DecompositionSolver solver = new QRDecomposition(A).getSolver();
        RealVector h = solver.solve(b);

        // Return the 3 parameters
        return h.toArray();
    }

    /**
     * Applies the homography transformation to a given (cameraX, cameraY) to get real inches.
     */
    public double transform(double x, double y, double[] H) {
        // Compute x' = h0 * x + h1 * y + h2
        return H[0] + H[1]*x + H[2]*y +H[3]*x*x + H[4]*y + H[5]*x*y;
    }

}
