package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    public Servo claw1, left, right;
    double leftPos, rightPos, clawPos;
    public ElapsedTime timer = new ElapsedTime();




    public Claw(HardwareMap hw){
        left = hw.get(Servo.class, "leftServo");
        right = hw.get(Servo.class, "rightServo");
        claw1 = hw.get(Servo.class, "clawServo");
        leftPos =0.086;
        rightPos =-0.554;
        clawPos= 1.0;


    }

    public class OpenClaw implements Action {
        private boolean clawTimer;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw1.setPosition(0.0);
            if (!clawTimer) {
                clawTimer = true;
                timer.reset();
            }

            clawPos = 1.0;
            return !clawTimer || !(timer.seconds() > 1);
        }
    }

    public class OpenClawPerm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw1.setPosition(0.0);
            clawPos = 0.0;
            return false;
        }
    }

    public Action openPerm(){
        return new OpenClawPerm();
    }

    public Action open(){
        return new OpenClaw();
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw1.setPosition(1.0);
            clawPos = 1.0;
            return false;
        }
    }

    public Action close(){
        return new CloseClaw();
    }

    public class MoveDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos = 0.266;
            rightPos = 1.0;

            left.setPosition(leftPos);
            right.setPosition(rightPos);
            return false;
        }
    }

    public Action moveDown(){
        return new MoveDown();
    }

    public class Hold implements Action{
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            claw1.setPosition(clawPos);
            return false;
        }

    }

    public Action hold(){
        return new Hold();
    }

    public class MoveUp implements Action{
        private boolean timerStarted = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos =0.25;
            rightPos =0.364;
            left.setPosition(leftPos);
            right.setPosition(rightPos);

            if (!timerStarted) {
                timerStarted = true;
                timer.reset();
            }

            return !timerStarted || !(timer.seconds() > 1);
        }
    }

    public Action moveUp(){
        return new MoveUp();
    }

    public class MoveMiddle implements Action{
        private boolean timerStarted = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            leftPos =-0.086;
            rightPos =-0.554;
            left.setPosition(leftPos);
            right.setPosition(rightPos);

            if (!timerStarted) {
                timerStarted = true;
                timer.reset();
            }

            return !timerStarted || !(timer.seconds() > 1);
        }
    }

    public Action moveMiddle(){
        return new MoveMiddle();
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