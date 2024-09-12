package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LinearSlideTest")
public class LinearSlide extends OpMode {
    public DcMotor linSlide;
    public ElapsedTime runtime;

    public void init(){
        linSlide = hardwareMap.get(DcMotor.class, "frontLeft");
        linSlide.setDirection(DcMotor.Direction.REVERSE);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runtime = new ElapsedTime();
        runtime.reset();
        // linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }

    public void loop(){
        double currentPos = 0.0;
        telemetry.addData("Runtime",runtime.seconds());
        telemetry.addData("Position", linSlide.getCurrentPosition());
        telemetry.update();
        double power = Math.abs(gamepad2.left_stick_y);
        if(gamepad2.left_stick_y<-0.1 && linSlide.getCurrentPosition()<3000){//go up on joystick/linear slide
            //currentPos += -gamepad1.left_stick_y*10;

            linSlide.setPower(power);
            //linSlide.setTargetPosition((int) currentPos);

        } else if(gamepad2.left_stick_y>0.1 && linSlide.getCurrentPosition()>5){ //go down on joystick/linear slide
            //currentPos += -gamepad1.left_stick_y*10;
            linSlide.setPower(-0.5);
            //linSlide.setTargetPosition((int) currentPos);
        } else{
            currentPos = linSlide.getCurrentPosition();
            linSlide.setPower(0.05);
            //linSlide.setTargetPosition((int) currentPos);
        }

    }
}