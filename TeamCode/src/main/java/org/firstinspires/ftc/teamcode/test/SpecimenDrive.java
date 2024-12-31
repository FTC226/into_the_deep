package org.firstinspires.ftc.teamcode.test;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;


/** Configuration Notes: CenterStage
 * Port 00: frontLeft
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 */

//@Disabled
@TeleOp (name = "Specimen Drive")
public class SpecimenDrive extends LinearOpMode {



    public class Drive { //subset
        public DcMotor frontLeft;
        public DcMotor frontRight;
        public DcMotor backLeft;
        public DcMotor backRight;

        public double botHeading;

        public IMU imu;
        HardwareMap hwMap;

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


        public Drive(HardwareMap ahwMap, Telemetry telem) {

            hwMap = ahwMap;
            Telem = telem;

            frontLeft = hwMap.get(DcMotor.class, "frontLeft");
            frontRight = hwMap.get(DcMotor.class, "frontRight");
            backLeft = hwMap.get(DcMotor.class, "backLeft");
            backRight = hwMap.get(DcMotor.class, "backRight");


            imu = hwMap.get(IMU.class, "imu");
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

        }

        public class DriveFC implements Action {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            boolean resetIMU = gamepad1.x;
            double[] wheelPower = new double[4];

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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

//        // FTC Dashboard
                Telem.addData("Target: ", Math.toDegrees(targetYaw));
                Telem.addData("Actual: ", Math.toDegrees(robotYaw));
                Telem.addData("Error: ", Math.toDegrees(error));
                Telem.addData("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
                Telem.update();

                return false;
            }
        }

        public Action driveFC(){
            return new DriveFC();
        }

        // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
        public double angleWrap(double radians) {

            while (radians > Math.PI) {
                radians -= 2 * Math.PI;
            }
            while (radians < -Math.PI) {
                radians += 2 * Math.PI;
            }

            // keep in mind that the result is in radians
            return radians;
        }
    }

    Robot robot;
    Drive drive;
    MecanumDrive drive2;
    Action clawCommand, armCommand, slideCommand;
    Action RunningCommand, DrivingCommand;
    ElapsedTime runtime = new ElapsedTime();

    public class Wait implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runtime.reset();
            while(runtime.seconds()<1){}
            return false;
        }
    }

    public Action Wait() {
        return new Wait();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = new Drive(hardwareMap, telemetry);

        Pose2d initialPose = new Pose2d(0, -34 , Math.toRadians(90.00));
//        Pose2d secondPose = new Pose2d(32.89, -53.98, Math.toRadians(-39.76));
//        Pose2d thirdPose = new Pose2d(32.89, -53.98, Math.toRadians(-90));
//        Pose2d finalPose = new Pose2d(32.89, -53.98, Math.toRadians(90));


        drive2 = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder goToHumanPlayer = drive2.actionBuilder(initialPose)
                .setReversed(true)

                .strafeToLinearHeading(new Vector2d(32.89, -53.98), Math.toRadians(-90))
                .waitSeconds(1)
                ;

        ;
        TrajectoryActionBuilder pickUpSpecimen = drive2.actionBuilder(new Pose2d(32.89, -53.98, Math.toRadians(-39.76)))
                .setReversed(true)
                .turn(Math.toRadians(-45))
                .waitSeconds(1)
                ;

        TrajectoryActionBuilder goToSub = drive2.actionBuilder(new Pose2d(32.89, -53.98, Math.toRadians(-90)))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-0, -34),Math.toRadians(-90))
                .waitSeconds(1)
                ;
        TrajectoryActionBuilder turnToSub = drive2.actionBuilder(new Pose2d(-0.22, -30.44,Math.toRadians(-90)))
                .turn(Math.toRadians(180))
                ;



        waitForStart();
        while (opModeIsActive()) {
            //RunningCommand = (gamepad2.a) ? robot.placeSample(): robot.holdPosition();
            //RunningCommand = (gamepad2.b) ? robot.resetPosition(): robot.holdPosition();


            if(gamepad2.a) { //move up sample
                RunningCommand = new ParallelAction(robot.placeSample(), goToHumanPlayer.build());
                drive.botHeading+=Math.toRadians(45);
//                RunningCommand = new ParallelAction(goToBucket.build());
            }
            else if(gamepad2.b) //move down to drive
                RunningCommand = robot.resetPosition();
            else if(gamepad2.x) {
                RunningCommand = new ParallelAction(robot.moveSub(), goToSub.build());
                drive.botHeading-=Math.toRadians(30);
//                RunningCommand = new ParallelAction(goToSub.build());
            }
            else if(gamepad2.y)
                RunningCommand = robot.score();
            else if(gamepad2.left_bumper)
                RunningCommand = robot.claw.close();
            else if(gamepad2.right_bumper)
                RunningCommand = robot.claw.openPerm();
            else
                RunningCommand = new ParallelAction(robot.holdPosition(), drive.driveFC());


            if(gamepad1.a) {
                DrivingCommand = goToHumanPlayer.build();
                drive.targetYaw -=Math.toRadians(135);
            }
            else if(gamepad1.b) {
                DrivingCommand = pickUpSpecimen.build();
                drive.targetYaw -=Math.toRadians(45);
            }
            else if(gamepad1.y) {
                DrivingCommand = goToSub.build();
                drive.targetYaw +=Math.toRadians(0);
            }
            else if(gamepad1.x) {
                DrivingCommand = turnToSub.build();
                drive.targetYaw +=Math.toRadians(180);
            }
            else
                DrivingCommand = drive.driveFC();

            //robot.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.x);

            Actions.runBlocking(new ParallelAction(
                        DrivingCommand
                    )
            );



        }

/*

clawCommand = (gamepad2.a) ? claw.open() : claw.close();
            armCommand = (gamepad2.b) ? arm.moveUp() : arm.hold();
            slideCommand = (gamepad2.x) ? slides.moveUp() : slides.hold();

            runningAction = (gamepad2.y) ?
                    new SequentialAction(arm.moveUp(), Wait(), Wait(), Wait(), Wait(), slides.moveUp()) :
                    new SequentialAction(arm.hold(),slides.hold());
if(gamepad2.b)
                armCommand = arm.moveUp();
            else
                armCommand = arm.hold();


    if(gamepad2.a){
                runningCommand = new SequentialAction(claw.open());
            } else{
                runningCommand = new SequentialAction(claw.close());
            }

            if(gamepad2.b){
                Actions.runBlocking(

                        new SequentialAction(
                                arm.moveUp(),
                                wait.build(),
                                slides.moveUp(),
                                wait.build()


                        )
                );
            }
            if(gamepad2.dpad_up){
                claw.moveUp();

        //new RunClaw(claw);

    }
            if(gamepad2.dpad_down){
        claw.moveDown();


    }

            if(gamepad2.right_stick_y > 0.1 && slidePosition < 2200){
        slidePower = gamepad2.right_stick_y;
        slidePosition += 1;
        slides.move(slidePosition, 1);
    }else if(gamepad2.right_stick_y < -0.1 && slidePosition > 0){
        slidePower = gamepad2.right_stick_y;
        slidePosition -= 1;
        slides.move(slidePosition, 1);
    } else{
        slides.move(slidePosition, 0);
    }

            if(gamepad2.left_stick_y > 0.1 && armPosition < 1700){
        armPower = gamepad2.right_stick_y;
        armPosition += 1;
        arm.move(armPosition, 1);
    } else if(gamepad2.left_stick_y < -0.1 && armPosition > 0){
        armPower = gamepad2.right_stick_y;
        armPosition -= 1;
        arm.move(armPosition, 1);
    }else{
        arm.move(armPosition, 0);
    }
            if(gamepad2.dpad_right){
        claw.rotate(true);
        //new RunClaw(claw);
    }
            if(gamepad2.dpad_left){
        claw.rotate(false);
        //new RunClaw(claw);
    }

}

 */
    }
}