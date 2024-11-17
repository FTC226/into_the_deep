package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Config
@TeleOp(name = "Final TeleOp")
public class mergeCode extends OpMode {

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    int armPos;

    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // Gamepad
    public double leftStickY, leftStickX, rightStickX;

    // PID for Field Centric
    public static double Kp = 0.5;
    public static double Ki = 0;
    public static double Kd = 0.1;

    public double targetYaw;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // FTC Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();



    public DcMotorEx arm;
    public DcMotorEx leftSlide, rightSlide;

    public long previousTime = 0;
    public double ePrevious = 0;
    public double eIntegral = 0;

    public static int target = 0;

    // PID for Linear Slide
    public static double p = 0.03, i = 0.3, d = 0.0002;

    //PID for arm
    public static double kp = 0.05, ki = 0.0, kd = 0.0;
    public static double kf = 0.1;

    public static int targetArm = 0;
    public static double armPower = 0;
    public static int slidePosition = 0;
    public static double servoPosition = 0;
    public static double clawPosition = 0;
    public static boolean slidesExtended = false;
    private final double ticks_in_degree = 700 / 180.0;

    public static int currentArmPosition =0;
    public static int currentSlidePosition=0;
    public static int intendedSlidePosition=0;







    //adaptiveClaw camera = new adaptiveClaw();

    Point center;
    double angle;
    double lat;
    double lon;



    public CRServo left, right;

    public Servo claw;
    public ElapsedTime runtime;
    //public double armPower;
    public double wristPower;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public double power;


    SampleDetection pipeline = new SampleDetection();


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);




        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide = hardwareMap.get(DcMotorEx .class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //controller = new PIDController(kp, ki, kd);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());






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

        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        runtime = new ElapsedTime();

        runtime.reset();

        imu.resetYaw();
        targetYaw = 0;
        integralSum = 0;
        lastError = 0;
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;


        if(gamepad2.dpad_left || gamepad2.dpad_right){//linear slide and arm code
            /*
            targetArm = 200;
            armPower = 0.5;
            slidePosition = 0;
            servoPosition = -1.0;

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);*/
            runtime.reset();
            while(runtime.milliseconds()<500){
                moveWrist(1.0, -1.0);
            }
            moveWrist(0.0, 0.0);
            currentSlidePosition = moveSlides(0);
            currentArmPosition = moveArm(400);
            //currentSlidePosition = moveSlides(500);



        } else if(gamepad2.dpad_up){//linear slide and arm code
            /*
            targetArm = 900;
            armPower = 0.7;
            slidePosition = 2200;
            servoPosition = 1.0;
            clawPosition = 0.0;

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);*/
            currentSlidePosition = moveSlides(0);
            currentArmPosition = moveArm(1700);

            try {
                Thread.sleep(750); // Delay for 500 milliseconds (0.5 seconds)
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            currentSlidePosition = moveSlides(2200);
            runtime.reset();


        } else if(gamepad2.dpad_down){
            /*
            targetArm = 0;
            armPower = 0.5;
            slidePosition = 0;
            servoPosition = -1.0;
            clawPosition = 0.0;

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);*/
            runtime.reset();
            while(runtime.milliseconds()<500){
                moveWrist(1.0, -1.0);
            }
            moveWrist(0.0, 0.0);
            currentSlidePosition = moveSlides(0);
            currentArmPosition = moveArm(0);
            currentSlidePosition = moveSlides(20);

        } else if(gamepad2.left_bumper){
            targetArm = 1000;
            armPower = 0.5;
            slidePosition = 0;

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);
        } else if(gamepad2.a){
            moveSlides(0);
            moveArm(2100);
            moveSlides(1000);
            moveArm(1900);

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);

            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1.0);
        }

        /*
        if(Math.abs(targetArm - arm.getCurrentPosition())>10 && (leftSlide.getCurrentPosition()>50
                || rightSlide.getCurrentPosition()>50)){ //sets the slides down before moving the arm
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(-1.0);
            rightSlide.setPower(-1.0);

            while(leftSlide.isBusy() || rightSlide.isBusy()){

            }
        }*/

        if(Math.abs(gamepad2.left_stick_y) > 0.1){
            moveWrist(gamepad2.left_stick_y, -gamepad2.left_stick_y);
        } else if(Math.abs(gamepad2.left_stick_x)>0.1) {
            moveWrist(gamepad2.left_stick_x, gamepad2.left_stick_x);
        } else {
            moveWrist(0.0,0.0);
        }

        if(currentArmPosition == 400){
            moveSlides(1400, -gamepad2.right_stick_y);
        }

        claw.setPosition(1- gamepad2.right_trigger);

        telemetry.addData("Arm Position", currentArmPosition);
        telemetry.addData("Slides Position", currentSlidePosition);



        if(gamepad2.x){ // moving to centralize the claw
            angle = pipeline.returnAngle();
            center = pipeline.returnCenter();

            double x = center.x;
            double y = center.y;
            if(Math.abs(x-320)>20 && Math.abs(y-180)>20){//setting the center
                moveRobotCentric(-(x-320)/Math.sqrt((x*x)+(y*y)), -(y-180)/Math.sqrt((x*x)+(y*y)), 0);
            } else if(Math.abs(angle-90)>3){
                power = (90-angle)/90;
                left.setPower(power);
                right.setPower(power); //setting the angle
            } else {
                claw.setPosition(1.0); //open the claw
            }


        } else { // move normally using gamepad1
            robotOrientation = imu.getRobotYawPitchRollAngles();
            robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

            double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
            double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

            //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
            if (Math.abs(rightStickX) > 0.1) {
                targetYaw = robotYaw;
                lastError = 0;
            }

            if (gamepad1.x) {
                imu.resetYaw();
                targetYaw = 0;
                integralSum = 0;
                lastError = 0;
            }

            // PID Calculations
            double error = targetYaw - robotYaw;
            error = (error + Math.PI) % (2 * Math.PI) - Math.PI;

            // Compute PID Terms
            double derivative = (error - lastError) / timer.seconds();
            integralSum += error * timer.seconds();
            double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            double rotationPower = Math.abs(rightStickX) > 0.1 ? rightStickX : -correction;

            frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
            frontRight.setPower((rotY - rotX - rotationPower) / denominator);
            backLeft.setPower((rotY - rotX + rotationPower) / denominator);
            backRight.setPower((rotY + rotX - rotationPower) / denominator);

            lastError = error;
            timer.reset();

            packet.put("Target: ", Math.toDegrees(targetYaw));
            packet.put("Actual: ", Math.toDegrees(robotYaw));
            packet.put("Error: ", Math.toDegrees(error));
            packet.put("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));


            telemetry.addData("Angle: ", angle);
            telemetry.addData("Latitude: ", lat);
            telemetry.addData("Longitude: ", lon);
            telemetry.addData("Center: ", center);
            telemetry.addData("Xpower", leftStickX);

            telemetry.addData("Ypower", leftStickY);
        }


        /*
        robotOrientation = imu.getRobotYawPitchRollAngles();

        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);
        robotYaw = (robotYaw + Math.PI) % (2 * Math.PI) - Math.PI;

        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
        if (Math.abs(rightStickX) > 0.1) {
            targetYaw = robotYaw;
            lastError = 0;
        }

        if (gamepad1.x) {
            imu.resetYaw();
            targetYaw = 0;
            integralSum = 0;
            lastError = 0;
        }

        // PID Calculations
        double error = targetYaw - robotYaw;
        error = (error + Math.PI) % (2 * Math.PI) - Math.PI;

        if (Math.abs(error) < Math.toRadians(2)) { // 2Â° tolerance
            error = 0;
        }

        // Compute PID Terms
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        double rotationPower = Math.abs(rightStickX) > 0.1 ? rightStickX : -correction;

        frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
        frontRight.setPower((rotY - rotX - rotationPower) / denominator);
        backLeft.setPower((rotY - rotX + rotationPower) / denominator);
        backRight.setPower((rotY + rotX - rotationPower) / denominator);

        lastError = error;
        timer.reset();

        //telemetry.addData("Angle: ", angle);
        //telemetry.addData("Latitude: ", lat);
        //telemetry.addData("Longitude: ", lon);
        //telemetry.addData("Center: ", center);
        //telemetry.addData("Xpower", leftStickX);

        //telemetry.addData("Ypower", leftStickY);

        // FTC Dashboard
        telemetry.update();
        //dashboard.sendTelemetryPacket(packet);
         */
    }

    public int moveSlides(int position){
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

        while(leftSlide.isBusy() || rightSlide.isBusy()){
            leftStickX = 0;
            leftStickY = 0;
            rightStickX = 0;
        }

        return position;
    }
    public void moveSlides(int position, double speed){
        if(speed < 0){
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(Math.abs(speed));
            rightSlide.setPower(Math.abs(speed));
        } else if(speed>0){
            leftSlide.setTargetPosition(position);
            rightSlide.setTargetPosition(position);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(Math.abs(speed));
            rightSlide.setPower(Math.abs(speed));
        }
        currentSlidePosition = leftSlide.getCurrentPosition();

    }

    public void moveWrist(double speed1, double speed2){
        left.setPower(speed1);
        right.setPower(speed2);
    }

    public int moveArm(int position){
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.3);

        while(arm.isBusy()){
            leftStickX = 0;
            leftStickY = 0;
            rightStickX = 0;
        }
        return position;
    }

    public void moveRobotCentric(double x, double y, double rx){
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / d;
        double backLeftPower = (y - x + rx) / d;
        double frontRightPower = (y - x - rx) / d;
        double backRightPower = (y + x - rx) / d;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void holdSlidePosition(int targetPosition) {
        // Continuously apply power to hold the slides at the target position
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!gamepad2.dpad_down && !gamepad2.dpad_up) {  // Keep holding position until another button is pressed
            // Re-apply small corrective power if the slides are not at the target position
            if (leftSlide.getCurrentPosition() < targetPosition) {
                leftSlide.setPower(0.1);  // Adjust power as needed
            } else {
                leftSlide.setPower(0);
            }

            if (rightSlide.getCurrentPosition() < targetPosition) {
                rightSlide.setPower(0.1);  // Adjust power as needed
            } else {
                rightSlide.setPower(0);
            }
        }
    }

    public double pidController(int target, double p, double i, double d) {
        long currentTime = micros();
        double deltaT = ((double)(currentTime +- previousTime)) / 1.0e6;

        int e = rightSlide.getCurrentPosition() - target;
        double eDerivative = (e - ePrevious) / deltaT;
        eIntegral = eIntegral + e * deltaT;

        double u = (p * e) + (d * eDerivative) + (i * eIntegral);

        previousTime = currentTime;
        ePrevious = e;

        return u;
    }

    public long micros() {
        return System.nanoTime() / 1000;
    }

    public void moveMotor(double power) {
        rightSlide.setPower(power);
    }
}