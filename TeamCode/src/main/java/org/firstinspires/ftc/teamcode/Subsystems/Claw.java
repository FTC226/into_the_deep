package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo claw1, left, right;
    double leftPos, rightPos;


    public Claw(HardwareMap hw){
        left = hw.get(Servo.class, "leftServo");
        right = hw.get(Servo.class, "rightServo");
        claw1 = hw.get(Servo.class, "clawServo");


    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw1.setPosition(0.0);
            return claw1.getPosition()>0.05;
        }
    }

    public Action open(){
        return new OpenClaw();
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw1.setPosition(1.0);
            return claw1.getPosition()<0.95;
        }
    }

    public Action close(){
        return new CloseClaw();
    }

    public class MoveDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos = 1.0;
            rightPos = 1.0;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            return left.getPosition()<0.95 || right.getPosition()<0.95;
        }
    }

    public Action moveDown(){
        return new MoveDown();
    }

    public class MoveUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos =0.0;
            rightPos =0.0;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            return left.getPosition()>0.05 || right.getPosition()>0.05;
        }
    }

    public Action moveUp(){
        return new MoveUp();
    }

    public void bigClose(){
        claw1.setPosition(0.0);
    }

    public void bigOpen(){
        claw1.setPosition(1.0);
    }



    public void rotate(boolean direction){//using dpads
        if(direction){
            leftPos -=0.01;
            rightPos -=0.01;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        } else{
            leftPos +=0.05;
            rightPos +=0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        }

    }

    public void move(boolean direction){
        if(direction){
            leftPos -=0.01;
            rightPos +=0.01;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        } else{
            leftPos +=0.05;
            rightPos -=0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        }
    }




}