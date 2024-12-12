package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LinearSlideTest")
public class LinearSlideTest extends OpMode {

    public DcMotor linSlide1,linSlide2;
    public ElapsedTime runtime;
    public static double upperLimit = 2900;

    public void init(){
        linSlide1 = hardwareMap.get(DcMotor.class, "frontLeft");
        linSlide2 = hardwareMap.get(DcMotor.class, "frontRight");//intakeSlide

        //linSlide1.setDirection(DcMotor.Direction.REVERSE);
        //linSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linSlide2.setDirection(DcMotorSimple.Direction.REVERSE);
        // linSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        runtime = new ElapsedTime();
        runtime.reset();
    }

    public void loop(){
        double power = 1.5* Math.abs(gamepad2.left_stick_y);
        if((power>1.0)){
            power = 1.0;
        }

        telemetry.addData("Runtime",runtime.seconds());
        telemetry.addData("Power",power);
        telemetry.addData("Position", linSlide1.getCurrentPosition());
        telemetry.update();

        if(gamepad2.left_stick_y<-0.1 && linSlide1.getCurrentPosition()<upperLimit && linSlide2.getCurrentPosition()<upperLimit){//go up on joystick/linear slide
            //currentPos += -gamepad1.left_stick_y*10;

            linSlide1.setPower(power);
            linSlide2.setPower(-power);
            //linSlide.setTargetPosition((int) currentPos);

        } else if(gamepad2.left_stick_y>0.1 && linSlide1.getCurrentPosition()>5 && linSlide2.getCurrentPosition()>5){ //go down on joystick/linear slide
            //currentPos += -gamepad1.left_stick_y*10;
            linSlide1.setPower(-0.7);
            linSlide2.setPower(-0.7);

            //linSlide.setTargetPosition((int) currentPos);
        } else{
            linSlide1.setPower(0.05);
            linSlide2.setPower(0.05);

        }
    }
}