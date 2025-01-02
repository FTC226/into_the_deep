package org.firstinspires.ftc.teamcode.test;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
//@TeleOp (name = "DuongTeleOpMode")
public class TeleOpMode extends LinearOpMode {

    public class Drive { //subset



        public Drive(HardwareMap ahwMap, Telemetry telem) {


        }

        public class DriveFC implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


//        // FTC Dashboard

                Telem.update();

                return false;
            }
        }

        public Action driveFC(){
            return new TeleOpMode.Drive.DriveFC();
        }

        // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.

    }

    Drive drive;

    public DcMotor leftSlide, rightSlide;
    public DcMotorEx arm;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public double botHeading;

    public IMU imu;

    // IMU
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // PID
    public double Kp = 0.5;
    public double Ki = 0;
    public double Kd = 0.1;

    public double targetYaw;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();
    Telemetry Telem;




    @Override
    public void runOpMode() throws InterruptedException {
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");


        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /* linSlide.setDirection(DcMotor.Direction.REVERSE);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);




        boolean isExpand = false;
        boolean isUp = false;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        boolean resetIMU = gamepad1.x;
        double[] wheelPower = new double[4];

        waitForStart();
        while (opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();

            robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-robotYaw) - y * Math.sin(-robotYaw);
            double rotY = x * Math.sin(-robotYaw) + y * Math.cos(-robotYaw);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
            if (Math.abs(rx) > 0.1) {
                targetYaw = robotYaw;
                lastError = 0;
            }

            if (resetIMU) {
                imu.resetYaw();
                targetYaw = 0;
                integralSum = 0;
                lastError = 0;
            }

            // PID Calculations
            double error = angleWrap(targetYaw - robotYaw);


            if (Math.abs(error) < Math.toRadians(2)) { // 2° tolerance
                error = 0;
            }

            // Compute PID Terms
            double derivative = (error - lastError) / timer.seconds();
            integralSum += error * timer.seconds();
            double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            double rotationPower = Math.abs(rx) > 0.1 ? rx : -correction;

            frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
            wheelPower[0]= frontLeft.getPower();

            frontRight.setPower((rotY - rotX - rotationPower) / denominator);
            wheelPower[1]= frontRight.getPower();

            backLeft.setPower((rotY - rotX + rotationPower) / denominator);
            wheelPower[2]= backLeft.getPower();

            backRight.setPower((rotY + rotX - rotationPower) / denominator);
            wheelPower[3]= backRight.getPower();

            lastError = error;
            timer.reset();


            if (gamepad1.a && !isExpand) {
                leftSlide.setTargetPosition(2100);
                rightSlide.setTargetPosition(2100);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(0.8);
                rightSlide.setPower(0.8);
                isExpand = true;
            } else if (gamepad1.a){
                leftSlide.setTargetPosition(0);
                rightSlide.setTargetPosition(0);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(0.8);
                rightSlide.setPower(0.8);
                isExpand = false;
            }


            if (gamepad1.b && !isUp) {
                arm.setTargetPosition(1570);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.3);
            } else if (gamepad1.b) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.3);
            }

            telemetry.addData("Target: ", Math.toDegrees(targetYaw));
            telemetry.addData("Actual: ", Math.toDegrees(robotYaw));
            telemetry.addData("Error: ", Math.toDegrees(error));
            telemetry.addData("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
            telemetry.addData("Slides", leftSlide.getCurrentPosition());
            telemetry.addData("Slides", leftSlide.getCurrentPosition());
            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.update();
        }

    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;

    }
}





//package org.firstinspires.ftc.teamcode;
//
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//
///** Configuration Notes: CenterStage
// * Port 00: frontLeft
// * Port 01: frontRight
// * Port 02: backLeft
// * Port 03: backRight
// */
//
//@TeleOp (name = "DuongTeleOpMode")
//public class TeleOpMode extends LinearOpMode {
//
//
//
//    public class Drive { //subset
//        public DcMotor frontLeft;
//        public DcMotor frontRight;
//        public DcMotor backLeft;
//        public DcMotor backRight;
//
//        public double botHeading;
//
//        public IMU imu;
//        HardwareMap hwMap;
//
//        // IMU
//        public YawPitchRollAngles robotOrientation;
//        public double robotYaw;
//
//        // PID
//        public double Kp = 0.5;
//        public double Ki = 0;
//        public double Kd = 0.1;
//
//        public double targetYaw;
//        double integralSum = 0;
//        double lastError = 0;
//
//        ElapsedTime timer = new ElapsedTime();
//        Telemetry Telem;
//
//
//        public Drive(HardwareMap ahwMap, Telemetry telem) {
//
//            hwMap = ahwMap;
//            Telem = telem;
//
//            frontLeft = hwMap.get(DcMotor.class, "frontLeft");
//            frontRight = hwMap.get(DcMotor.class, "frontRight");
//            backLeft = hwMap.get(DcMotor.class, "backLeft");
//            backRight = hwMap.get(DcMotor.class, "backRight");
//
//
//            imu = hwMap.get(IMU.class, "imu");
//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
//                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//
//            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//            imu.initialize(parameters);
//
//            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//            frontLeft.setDirection(DcMotor.Direction.REVERSE);
//            backLeft.setDirection(DcMotor.Direction.REVERSE);
//            frontRight.setDirection(DcMotor.Direction.FORWARD);
//            backRight.setDirection(DcMotor.Direction.FORWARD);
//
//
//            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//       /* linSlide.setDirection(DcMotor.Direction.REVERSE);
//        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
//
//            frontLeft.setPower(0);
//            backLeft.setPower(0);
//            frontRight.setPower(0);
//            backRight.setPower(0);
//
//            targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        }
//
//        public class DriveFC implements Action {
//            double x = gamepad1.left_stick_x;
//            double y = -gamepad1.left_stick_y;
//            double rx = gamepad1.right_stick_x;
//            boolean resetIMU = gamepad1.x;
//            double[] wheelPower = new double[4];
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                robotOrientation = imu.getRobotYawPitchRollAngles();
//
//                robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);
//
//                double rotX = x * Math.cos(-robotYaw) - y * Math.sin(-robotYaw);
//                double rotY = x * Math.sin(-robotYaw) + y * Math.cos(-robotYaw);
//                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//
//                //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
//                if (Math.abs(rx) > 0.1) {
//                    targetYaw = robotYaw;
//                    lastError = 0;
//                }
//
//                if (resetIMU) {
//                    imu.resetYaw();
//                    targetYaw = 0;
//                    integralSum = 0;
//                    lastError = 0;
//                }
//
//                // PID Calculations
//                double error = angleWrap(targetYaw - robotYaw);
//
//                if (Math.abs(error) < Math.toRadians(2)) { // 2° tolerance
//                    error = 0;
//                }
//
//                // Compute PID Terms
//                double derivative = (error - lastError) / timer.seconds();
//                integralSum += error * timer.seconds();
//                double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//                double rotationPower = Math.abs(rx) > 0.1 ? rx : -correction;
//
//                frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
//                wheelPower[0]= frontLeft.getPower();
//
//                frontRight.setPower((rotY - rotX - rotationPower) / denominator);
//                wheelPower[1]= frontRight.getPower();
//
//                backLeft.setPower((rotY - rotX + rotationPower) / denominator);
//                wheelPower[2]= backLeft.getPower();
//
//                backRight.setPower((rotY + rotX - rotationPower) / denominator);
//                wheelPower[3]= backRight.getPower();
//
//                lastError = error;
//                timer.reset();
//
////        // FTC Dashboard
//                Telem.addData("Target: ", Math.toDegrees(targetYaw));
//                Telem.addData("Actual: ", Math.toDegrees(robotYaw));
//                Telem.addData("Error: ", Math.toDegrees(error));
//                Telem.addData("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
//                Telem.update();
//
//                return false;
//            }
//        }
//
//        public Action driveFC(){
//            return new DriveFC();
//        }
//
//        // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
//        public double angleWrap(double radians) {
//
//            while (radians > Math.PI) {
//                radians -= 2 * Math.PI;
//            }
//            while (radians < -Math.PI) {
//                radians += 2 * Math.PI;
//            }
//
//            // keep in mind that the result is in radians
//            return radians;
//        }
//    }
//
//    Robot robot;
//    Drive drive;
//    MecanumDrive drive2;
//    Action clawCommand, armCommand, slideCommand;
//
//    Action RunningCommand = new ParallelAction();
//
//
//
//    ElapsedTime runtime = new ElapsedTime();
//
//    public class Wait implements Action {
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            runtime.reset();
//            while(runtime.seconds()<1){}
//            return false;
//        }
//    }
//
//    public Action Wait() {
//        return new Wait();
//    }
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot(hardwareMap, telemetry);
//        drive = new Drive(hardwareMap, telemetry);
//
//        waitForStart();
//        while (opModeIsActive()) {
//
//            Action driveAction = drive.driveFC();
//
//
//
//            if (gamepad2.a) {
//                RunningCommand = robot.placeSampleTest();
//            } else if (gamepad2.b)
//                RunningCommand = robot.resetPositionTest();
//            else if (gamepad2.x) {
//                RunningCommand = robot.moveSubTest();
//            } else if (gamepad2.y)
//                RunningCommand = robot.scoreTest();
//            else if (gamepad2.left_bumper)
//                RunningCommand = robot.claw.close();
//            else if (gamepad2.right_bumper)
//                RunningCommand = robot.claw.openPerm();
//            else
//                RunningCommand = new ParallelAction(robot.holdPosition(), drive.driveFC());
//
//
//            Actions.runBlocking(new ParallelAction(RunningCommand, driveAction)
//            );
//
//
//        }
//    }
//}