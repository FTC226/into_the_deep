package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SampleDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import org.opencv.core.*;


@TeleOp(name = "Claw Mover Tester")
public class clawMovement extends OpMode {

    adaptiveClaw camera = new adaptiveClaw();

    Point center;
    double angle;
    double lat;
    double lon;

    public CRServo left, right;

    public Servo claw;
    public ElapsedTime runtime;
    public double armPower;
    public double wristPower;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public double power;

    SampleDetection pipeline = new SampleDetection();

    public void init(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);


        left = hardwareMap.get(CRServo.class, "leftServo");
        right = hardwareMap.get(CRServo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        runtime = new ElapsedTime();

        runtime.reset();

        /*
        while(runtime.milliseconds()<1000){
            left.setPosition(0.0);
            right.setPosition(0.0);
        }

        while(runtime.milliseconds()<5000){
            left.setPosition(1.0);
            right.setPosition(1.0);
        }

        while(runtime.milliseconds()<10000){
            left.setPosition(-1.0);
            right.setPosition(-1.0);
        }

        left.setPosition(0.0);
        right.setPosition(0.0);
        */
    }

    public void loop(){
        angle = pipeline.returnAngle();
        center = pipeline.returnCenter();

        runtime.reset();

        armPower = -gamepad2.left_stick_y;
        wristPower = gamepad2.left_stick_x;



        if(armPower > 0.5){
            left.setPower(-armPower);
            right.setPower(armPower);
        }

        else if(armPower < -0.5){

            left.setPower(-armPower);
            right.setPower(armPower);

        }

        else if(wristPower > 0.5){
            left.setPower(armPower);
            right.setPower(armPower);

        }
        else if(wristPower < -0.5){
            left.setPower(-armPower);
            right.setPower(-armPower);
        } else{
            left.setPower(0);
            right.setPower(0);
        }

        if (gamepad2.a) {

        }

        if(gamepad2.b){

        }

        if(gamepad2.x){
            power = (90-angle)/90;
            if (Math.abs(power)<(0.06));

            left.setPower(power);
            right.setPower(power);
        }

        if(gamepad2.y){
            claw.setPosition(1.0);
        }
        else{
            claw.setPosition(0.0);
        }

        telemetry.addData("Angle: ", angle);
        telemetry.addData("Latitude: ", lat);
        telemetry.addData("Longitude: ", lon);
        telemetry.addData("Center: ", center);
        telemetry.update();
    }
}
