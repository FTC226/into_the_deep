package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LinearSlideTest")
public class LinearSlideTest extends OpMode {

    public DcMotor linSlide;
    public ElapsedTime runtime;
    public static double upperLimit = 2900;

    public void init(){
        linSlide = hardwareMap.get(DcMotor.class, "intakeSlide");

        linSlide.setDirection(DcMotor.Direction.REVERSE);
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime = new ElapsedTime();
        runtime.reset();
    }

    public void loop(){
        telemetry.addData("Runtime",runtime.seconds());
        telemetry.addData("Position", linSlide.getCurrentPosition());
        telemetry.update();
        double power = 1.5* Math.abs(gamepad2.left_stick_y);
        if((power>1.0)){
            power = 1.0;
        }
        if(gamepad2.left_stick_y<-0.1 && linSlide.getCurrentPosition()<upperLimit){//go up on joystick/linear slide
            //currentPos += -gamepad1.left_stick_y*10;

            linSlide.setPower(power);
            //linSlide.setTargetPosition((int) currentPos);

        } else if(gamepad2.left_stick_y>0.1 && linSlide.getCurrentPosition()>5){ //go down on joystick/linear slide
            //currentPos += -gamepad1.left_stick_y*10;
            linSlide.setPower(-0.7);
            //linSlide.setTargetPosition((int) currentPos);
        } else{
            linSlide.setPower(0.05);
        }
    }
}