package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LinearSlideTesting2 extends OpMode {

    public int targetPos = 0;
    double kpLeft = 1; //needs tuning
    double kpRight = 1; //needs tuning

    public static int upperLimit = 3000;

    DcMotor linearSlideRight;
    DcMotor linearSlideLeft;

    public void init(){
        linearSlideLeft = hardwareMap.get(DcMotor.class,"intakeSlideLeft");
        linearSlideRight = hardwareMap.get(DcMotor.class,"intakeSlideRight");

        linearSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        moveSlides(gamepad2.left_stick_y);
        telemetry.addData("Left Side Position", linearSlideLeft.getCurrentPosition());
        telemetry.addData("Right Side Position", linearSlideRight.getCurrentPosition());

        telemetry.addData("Left Side Power", kpLeft*gamepad2.left_stick_y);
        telemetry.addData("Right Side Power", kpRight*gamepad2.left_stick_y);
    }

    public void moveSlides(double leftStickY){
        if(leftStickY>0.1){
            linearSlideLeft.setTargetPosition(upperLimit);
            linearSlideRight.setTargetPosition(upperLimit);

            linearSlideLeft.setPower(kpLeft*leftStickY);
            linearSlideRight.setPower(kpRight*leftStickY);
        } else if(leftStickY<0.1){
            linearSlideLeft.setTargetPosition(0);
            linearSlideRight.setTargetPosition(0);

            linearSlideLeft.setPower(kpLeft*leftStickY);
            linearSlideRight.setPower(kpRight*leftStickY);
        } else {
            linearSlideLeft.setTargetPosition(linearSlideLeft.getCurrentPosition());
            linearSlideRight.setTargetPosition(linearSlideLeft.getCurrentPosition());

            linearSlideLeft.setPower(0.05);
            linearSlideRight.setPower(0.05);
        }
    }
}
