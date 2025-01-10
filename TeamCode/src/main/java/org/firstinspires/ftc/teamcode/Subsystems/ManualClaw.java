package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ManualClaw {
    public Servo claw1, left, right;
    double leftPos, rightPos, clawPos;
    public ElapsedTime timer = new ElapsedTime();




    public ManualClaw(HardwareMap hw){
        left = hw.get(Servo.class, "leftServo");
        right = hw.get(Servo.class, "rightServo");
        claw1 = hw.get(Servo.class, "clawServo");
        leftPos = left.getPosition();
        rightPos = right.getPosition();

    }

    public void moveClaw(boolean direction){
        if(direction){
            leftPos = leftPos - 0.05;
            rightPos = rightPos + 0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        } else{
            leftPos = leftPos + 0.05;
            rightPos = rightPos - 0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        }
    }

    public void turn (boolean direction){
        if(direction){
            leftPos = leftPos - 0.05;
            rightPos = rightPos - 0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        } else{
            leftPos = leftPos + 0.05;
            rightPos = rightPos + 0.05;
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        }
    }

    public void open(){
        claw1.setPosition(0);
    }

    public void close(){
        claw1.setPosition(1);
    }
}
