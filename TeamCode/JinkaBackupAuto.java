package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "JinkaBackupAuto")
public class JinkaBackupAuto extends LinearOpMode {
    // need to make parameter for choosing left side or right side when we start
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor arm;
    private Servo armClaw;
    private DcMotor slide;
    private Servo slideClaw;
    private static final double CLAW_OPEN = 0;
    private static final double CLAW_CLOSE = 0.8;
    //Add linear slide

    // Constant for encoder ticks per tile (assuming 1060 ticks per tile)
    private static final int TICKS_PER_TILE = 1060;


    @Override
    public void runOpMode() {
        // Initialize motors
        //initializations checked
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        arm = hardwareMap.get(DcMotor.class, "arm");
        armClaw = hardwareMap.get(Servo.class, "armClaw");

        slide = hardwareMap.get(DcMotor.class, "slide");
        slideClaw = hardwareMap.get(Servo.class, "slideClaw");



        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the driver to press start

        waitForStart();

        //add auto methods here

        //testing methods

        moveForward(1000);
        moveBackward(1000);
        moveLeft(1000);
        moveRight(1000);
        clockwise(1000);
        counterClockwise(1000);


        moveArm(500);
        moveArm(-500);

        moveSlide(100);
        moveSlide(-100);


        armClawOpen();
        armClawClose();

        slideClawOpen();
        slideClawClose();
    }



    ///////////////slide and arm



    //checked
    // Method to move forward a certain distance (in ticks)
    public void moveArm(int ticks) {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        arm.setTargetPosition(ticks); // Set target position
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Enable encoder control
        arm.setPower(0.5); // Set power for arm

        // Wait for arm to reach its target
        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Arm Status", "Moving...");
            telemetry.update();
            sleep(50); // Prevent loop from running too fast
        }

        arm.setPower(0); // Stop arm motor
    }

    //checked
    //making new linearSlide method
    public void moveSlide(int ticks){
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(ticks);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.5);

        //Wait for linear slide to reach its target
        while (opModeIsActive() && slide.isBusy()){
            telemetry.addData("Slide Status", "Moving...");
            telemetry.update();
            sleep(50); // Prevent loop from running too fast
        }

        slide.setPower(0); // Stop slide motor
    }

    //checked
    // Method to open armClaw
    public void armClawOpen() {
        armClaw.setPosition(CLAW_OPEN);
        telemetry.addData("ArmClaw", "Open");
        telemetry.update();
    }
    //checked
    // Method to close armClaw
    public void armClawClose() {
        armClaw.setPosition(CLAW_CLOSE);
        telemetry.addData("ArmClaw", "Closed");
        telemetry.update();
    }

    //checked
    //Method to open slideClaw
    public void slideClawOpen(){
        slideClaw.setPosition(CLAW_OPEN);
        telemetry.addData("SlideClaw", "Open");
        telemetry.update();
    }

    //checked
    //Method to close slideClaw
    public void slideClawClose(){
        slideClaw.setPosition(CLAW_CLOSE);
        telemetry.addData("SlideClaw", "Closed");
        telemetry.update();
    }







    //////////Movement


    //checked
    // Method to move forward
    public void moveForward(int distance) {
        moveMotors(-distance, -distance, -distance, -distance);
    }

    //checked
    // Method to move backward
    public void moveBackward(int distance) {
        moveMotors(distance, distance, distance, distance);
    }

    //checked
    // Method to move right
    public void moveLeft(int distance) {
        moveMotors(distance, -distance, distance, -distance);
    }

    //checked
    // Method to move left
    public void moveRight(int distance) {
        moveMotors(-distance, distance, -distance, distance);
    }

    public void clockwise(int distance){
        moveMotors(distance, -distance, -distance, distance);
    }

    public void counterClockwise(int distance){
        moveMotors(-distance, distance, distance, -distance);
    }

    //checked
    // Method to move motors by setting their target positions
    public void moveMotors(int leftBackTarget, int rightBackTarget, int leftFrontTarget, int rightFrontTarget) {
        resetEncoders(); // Reset encoders before setting the target positions

        // Set target positions
        frontLeft.setTargetPosition(leftFrontTarget);
        frontRight.setTargetPosition(rightFrontTarget);
        backLeft.setTargetPosition(leftBackTarget);
        backRight.setTargetPosition(rightBackTarget);

        // Set motors to run to the target position
        setAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power to move
        setMotorPower(0.6);

        // Wait until motors reach their target
        waitForMotors();

        // Stop motors once the target is reached
        stopMotors();
    }







    ////////////Helper methods



    //checked
    // Method to reset motor encoders
    public void resetEncoders() {
        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //checked
    // Method to wait until both motors reach their target position
    public void waitForMotors() {
        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy()|| backRight.isBusy()|| backLeft.isBusy())) {
            telemetry.addData("Status", "Motors Moving...");
            telemetry.update();
            sleep(50); // Small delay to prevent loop from running too fast
        }
    }

    //checked
    // Helper method to set the same mode for both motors
    public void setAllMotorsMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);

    }

    //checked
    // Helper method to set the power for all motors
    public void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    //checked
    // Method to stop both motors
    public void stopMotors() {
        setMotorPower(0);
    }
}