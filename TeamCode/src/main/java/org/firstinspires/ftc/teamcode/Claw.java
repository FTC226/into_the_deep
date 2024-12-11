package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
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
            claw1.setPosition(1.0);
            return false;
        }
    }

    public Action open(){
        return new OpenClaw();
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw1.setPosition(0.0);
            return false;
        }
    }

    public Action close(){
        return new CloseClaw();
    }

    public class MoveDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos += 0.5;
            rightPos += 0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            return false;
        }
    }

    public Action moveDown(){
        return new MoveDown();
    }

    public class MoveUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos -= 0.05;
            rightPos -= 0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            return false;
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
