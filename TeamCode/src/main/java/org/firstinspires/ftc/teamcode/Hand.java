package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hand{
    public CRServo leftServo;
    public CRServo rightServo;
    public Servo pincher;

    HardwareMap hw;

    public void init(HardwareMap ahwMap){
        this.hw = ahwMap;

        leftServo = hw.get(CRServo.class, "leftServo");
        rightServo = hw.get(CRServo.class, "rightServo");
        pincher = hw.get(Servo.class, "pinchServo");

    }

    public void moveWrist(double leftPower, double rightPower){
        leftServo.setPower(leftPower);
        rightServo.setPower(rightPower);
    }

    public void pinch(boolean p){
        if(p){
            pincher.setPosition(0.0);
        } else {
            pincher.setPosition(1.0);
        }
    }

    public double returnServoStates(Object s){ //similar to the toString method
        if (s instanceof Servo){
            return ((Servo)s).getPosition();
        } else if (s instanceof CRServo){
            return ((CRServo)s).getPower();
        }
        return -2;
    }
}
