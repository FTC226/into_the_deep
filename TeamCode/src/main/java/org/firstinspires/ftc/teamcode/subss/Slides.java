package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    public Encoder leftSlideEncoder;
    public Encoder rightSlideEncoder;

    ElapsedTime timer = new ElapsedTime();

    public boolean isReset = false;

    OpMode opMode;

    int velocity = 5000; //4250

    public Slides(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftSlide = opMode.hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = opMode.hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlideEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "leftSlide"));
        rightSlideEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "rightSlide"));
        rightSlideEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public void pickupSample() {
        leftSlide.setTargetPosition(1300);
        rightSlide.setTargetPosition(1300);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void hangExtend(){
        leftSlide.setTargetPosition(1100);
        rightSlide.setTargetPosition(1100);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void placeSample() {
        leftSlide.setTargetPosition(2300);
        rightSlide.setTargetPosition(2300);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void placeSampleLow() {
        leftSlide.setTargetPosition(550);
        rightSlide.setTargetPosition(550);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void moveToResetPos() {
        leftSlide.setTargetPosition(10);
        rightSlide.setTargetPosition(10);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void move(int ticks) {
        leftSlide.setTargetPosition(ticks);
        rightSlide.setTargetPosition(ticks);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(velocity);
        rightSlide.setVelocity(velocity);
    }

    public void stop() {
//        leftSlide.setTargetPosition(0);
//        rightSlide.setTargetPosition(0);
//        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setVelocity(0);
        rightSlide.setVelocity(0);
        resetEncoder();
    }

    public int leftGetCurrentPosition() {
        return leftSlide.getCurrentPosition();
    }

    public int rightGetCurrentPosition() {
        return rightSlide.getCurrentPosition();
    }

    public void resetSlides() {
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setVelocity(-3000);
        rightSlide.setVelocity(-3000);
    }

    public void resetEncoder() {
        leftSlideEncoder.reset();
        rightSlideEncoder.reset();
    }
}