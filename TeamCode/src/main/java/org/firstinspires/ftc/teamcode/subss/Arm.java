package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm {
    DcMotorEx arm;
    public Encoder armEncoder;
    OpMode opMode;
    double power;

    ElapsedTime timer = new ElapsedTime();

    public boolean isReset = false;

    int velocity = 5000; //4250


    public Arm(OpMode _opMode, double power) {
        opMode = _opMode;
        this.power = power;
    }

    public void init() {
        arm = opMode.hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armEncoder = new Encoder(opMode.hardwareMap.get(DcMotorEx.class, "arm"));
        armEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public void moveUp() {
        arm.setTargetPosition(1750);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);
    }

    public void moveDown() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);
    }

    public void readyForHang() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);
    }

    public int getCurrentPosition() {
        return arm.getCurrentPosition();
    }

    public void stop() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(0);
        resetEncoder();
    }

    public void resetArm() {
        arm.setTargetPosition(0);
        arm.setVelocity(-1500);
    }

    public void resetEncoder() {
        armEncoder.reset();
    }

}
