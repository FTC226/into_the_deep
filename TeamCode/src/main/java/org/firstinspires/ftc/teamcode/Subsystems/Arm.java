package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotorEx arm;
    public int targetPosition;


    public Arm(HardwareMap hw){
        arm = hw.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class MoveUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 1700;
            arm.setTargetPosition(1700);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            return Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())>10;
        }

    }

    public Action moveUp(){
        return new MoveUp();
    }

    public class Hold implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            arm.setTargetPosition(targetPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            return false;
        }

    }

    public Action hold(){
        return new Hold();
    }

    public class MoveDown implements  Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 0;
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            return Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())>10;
        }
    }

    public Action moveDown(){
        return new MoveDown();
    }


    public void move(int position, double power){

        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }




}
