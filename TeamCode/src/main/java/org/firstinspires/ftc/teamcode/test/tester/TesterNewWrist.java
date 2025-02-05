package org.firstinspires.ftc.teamcode.test.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Claw;
import org.firstinspires.ftc.teamcode.subss.Wrist;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Wrist Tester1")
public class TesterNewWrist extends LinearOpMode {
    public Camera camera = new Camera(this);
    public Wrist wrist = new Wrist(this);
    private OpenCvCamera webcam;
    public Claw claw = new Claw(this);
    Mat temp;

    @Override
    public void runOpMode() throws InterruptedException {

        claw.init();
        camera.init();
        FtcDashboard dashboard = FtcDashboard.getInstance();


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);



        // Initialize the pipeline

        webcam.setPipeline(camera);




        waitForStart();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                dashboard.startCameraStream(webcam, 30);
                telemetry.addData("Status", "Camera started");

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                telemetry.update();
            }
        });


        while(opModeIsActive()){
            /*
            temp = camera.processFrame(camera.frame);
            telemetry.addData("Angle", camera.realAngle());
            telemetry.addData("Center X", camera.realX());
            telemetry.addData("Center Y", camera.realY());

             */

            if(gamepad1.b) { // moving to centralize the claw
                if (camera.realAngle() > 67.5 || camera.realAngle() < 22.5) {
                    wrist.PickUp90();
                } else {
                    wrist.PickUp0();
                }
            }

            telemetry.addData("Angle", camera.realAngle());
            telemetry.addData("Center X", camera.realX());
            telemetry.addData("Center Y", camera.realY());


            telemetry.update();
        }

    }
}
