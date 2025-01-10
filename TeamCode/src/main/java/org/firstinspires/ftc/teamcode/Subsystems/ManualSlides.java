package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ManualSlides {
    public DcMotor leftSlide, rightSlide;
    public int targetPosition;
    Telemetry Telem;
    private ElapsedTime timer = new ElapsedTime();


    public ManualSlides(HardwareMap hw, Telemetry tm){
        Telem = tm;
        leftSlide = hw.get(DcMotor.class, "leftSlide");
        rightSlide = hw.get(DcMotor.class, "rightSlide");


        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPosition= rightSlide.getCurrentPosition();
    }

    public void moveSlides(double position){
        targetPosition = targetPosition - ((int)position*5);

        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);
        Telem.addData("Left Slides", leftSlide.getCurrentPosition());
        Telem.addData("Right Slides", rightSlide.getCurrentPosition());
        Telem.update();
    }

    public void hold(){
        targetPosition = rightSlide.getCurrentPosition();
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
    }
}
