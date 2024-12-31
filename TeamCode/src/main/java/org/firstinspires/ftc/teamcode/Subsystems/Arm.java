package org.firstinspires.ftc.teamcode.subsystems;

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
        arm.setTargetPosition(1600);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }

    public void moveDown() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }


    public int getCurrentPosition() {
        return arm.getCurrentPosition();
    }

    public void resetEncoder() {
        armEncoder.reset();
    }


    public void resetArm() {
        ElapsedTime timerRestArm = new ElapsedTime();
        arm.setTargetPosition(0);
        arm.setPower(-1);
        while (timerRestArm.seconds() > 3) {
            arm.setPower(0);
            armEncoder.reset();
        }
    }

    private boolean armReachedTarget(int targetArm, int threshold) {
        return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
    }
}
