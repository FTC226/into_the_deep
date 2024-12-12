package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

public class Slides {
    public DcMotorEx leftSlide, rightSlide;
    public int targetPosition;


    public Slides(HardwareMap hw){
        leftSlide = hw.get(DcMotorEx .class, "leftSlide");
        rightSlide = hw.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class MoveUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 900;
            leftSlide.setTargetPosition(900);
            rightSlide.setTargetPosition(900);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
            return Math.abs(leftSlide.getTargetPosition()-leftSlide.getCurrentPosition())>10;        }
    }

    public Action moveUp(){
        return new MoveUp();
    }

    public class Hold implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){

            leftSlide.setTargetPosition(targetPosition);
            rightSlide.setTargetPosition(targetPosition);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(0.1);
            rightSlide.setPower(0.1);
            return false;
        }

    }

    public Action hold(){
        return new Hold();
    }

    public class MoveDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){

            leftSlide.setTargetPosition(5);
            rightSlide.setTargetPosition(5);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
            return Math.abs(leftSlide.getTargetPosition()-leftSlide.getCurrentPosition())<10;
        }
    }

    public Action moveDown(){
        return new MoveDown();
    }



    public void move(int position, double power){
        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }


}
