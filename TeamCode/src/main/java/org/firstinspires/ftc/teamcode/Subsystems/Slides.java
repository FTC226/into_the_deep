package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slides {
    public DcMotor leftSlide, rightSlide;
    public int targetPosition;
    Telemetry Telem;
    private ElapsedTime timer = new ElapsedTime();


    public Slides(HardwareMap hw, Telemetry tm){
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
    }

    public class MoveUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 2100;
            leftSlide.setTargetPosition(2100);
            rightSlide.setTargetPosition(2100);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(0.8);
            rightSlide.setPower(0.8);
            Telem.addData("Slides", leftSlide.getCurrentPosition());


            //leftSlide.setPower((2200-leftSlide.getCurrentPosition())/2200);
            //rightSlide.setPower((2200-leftSlide.getCurrentPosition())/2200);
            //telemetryPacket.put("Encoder Value", leftSlide.getTargetPosition());
            Telem.addData("Slides", leftSlide.getCurrentPosition());
            Telem.update();
            return Math.abs(leftSlide.getTargetPosition()-leftSlide.getCurrentPosition())>5;

        }
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
            leftSlide.setPower(0.8);
            rightSlide.setPower(0.8);

            return false;
        }

    }

    public Action hold(){
        return new Hold();
    }

    public class MoveDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition=0;

            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
            Telem.addData("Left Slides", leftSlide.getCurrentPosition());
            Telem.addData("Right Slides", rightSlide.getCurrentPosition());
            Telem.update();
            return Math.abs(leftSlide.getTargetPosition()-leftSlide.getCurrentPosition())>50;
        }
    }

    public Action moveDown(){
        return new MoveDown();
    }

    public class MoveSub implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition=900;

            leftSlide.setTargetPosition(900);
            rightSlide.setTargetPosition(900);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
            Telem.addData("Left Slides", leftSlide.getCurrentPosition());
            Telem.addData("Right Slides", rightSlide.getCurrentPosition());
            Telem.update();
            return Math.abs(leftSlide.getTargetPosition()-leftSlide.getCurrentPosition())>50;
        }
    }

    public Action moveSub(){
        return new MoveSub();
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