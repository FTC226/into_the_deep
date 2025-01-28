package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;


public class cameraProgram implements VisionProcessor{
    public Rect rectLeft = new Rect(0, 160, 140, 160);
    public Rect rectMiddle = new Rect(214, 160, 213, 160);
    public Rect rectRight = new Rect(500, 160, 140, 160);
    public double blueRectLeft;
    public double blueRectMiddle;
    public double blueRectRight;
    int cnt=0;

    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration){
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        //Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        blueRectLeft = getAvgBlueness(frame, rectLeft);
        blueRectMiddle = getAvgBlueness(frame, rectMiddle);
        blueRectRight = getAvgBlueness(frame, rectRight);

/*
        // Get the BGR values of the pixel at position (x, y)
        double[] bgrValues = frame.get(0, 0);
        redRectLeft = frame.channels(); //bgrValues[0];
        redRectMiddle = bgrValues[1];
        redRectRight = bgrValues[2];
*/
        if ((blueRectLeft > blueRectMiddle)&&(blueRectLeft > blueRectRight)){
            return Selected.LEFT;
        } else if ((blueRectMiddle>blueRectLeft)&&(blueRectMiddle>blueRectRight)){
            return Selected.MIDDLE;
        }

        return Selected.RIGHT;
    }

    protected double getAvgBlueness(Mat input, Rect rect){
        Mat submat = input.submat(rect);
        Scalar lowerBlue = new Scalar(0, 0, 75, 0);
        Scalar upperBlue = new Scalar(50, 50, 255, 255);

        Mat mask = new Mat();
        Core.inRange(submat, lowerBlue, upperBlue, mask);

                /*
        // Calculate the percentage of red pixels
        int totalPixels = submat.rows() * submat.cols();
        int redPixels = Core.countNonZero(mask);
        double rednessPercentage = (redPixels / (double) totalPixels) * 100;
        return rednessPercentage;
        */

        double meanBlue = Core.mean(submat, mask).val[0];
        return meanBlue;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onScreenWidth, int onScreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonselectedPaint = new Paint(selectedPaint);
        nonselectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection){
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonselectedPaint);
                canvas.drawRect(drawRectangleRight, nonselectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonselectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonselectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonselectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonselectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonselectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonselectedPaint);
                canvas.drawRect(drawRectangleRight, nonselectedPaint);
                break;
        }
    }

    public Selected getSelection(){
        return selection;
    }

    public enum Selected{
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }

}


