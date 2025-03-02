package org.firstinspires.ftc.teamcode.opmodes.test.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Wrist;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp (name = "Tester - Wrist Cam")
public class TesterWristCam extends OpMode {

    private OpenCvCamera webcam;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    Camera camera = new Camera(this);
    Wrist wrist = new Wrist(this);

    double realAngle;


    @Override
    public void init() {
        camera.init();
        wrist.init();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(camera);

        camera.setColor("blue");

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

                dashboard.startCameraStream(webcam, 60);
                telemetry.addData("Status", "Camera started");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                telemetry.update();
            }
        });

        wrist.middleWrist();
    }

    @Override
    public void loop() {
        wrist.setRotateServo(angleOrientation());
    }


    public double angleOrientation(){
        realAngle = camera.realAngle();
        if (realAngle > 0) {
            return (0.00355556*realAngle+0.16);
        }
        else {
            return (0.004*realAngle+0.84);
        }
    }

}