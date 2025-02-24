package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Wrist {
    public Servo wristServo;
    public Servo rotateServo;
    OpMode opMode;

    public Wrist(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        wristServo = opMode.hardwareMap.get(Servo.class, "wristServo"); //leftServo
        rotateServo = opMode.hardwareMap.get(Servo.class, "rotateServo"); //rightServo
    }


    public void Up() {
        wristServo.setPosition(0.2);
        rotateServo.setPosition(0.48);
    }

    public void Down() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.48);
    }

    public void PlaceSample() {
        wristServo.setPosition(0.2); //0.45
        rotateServo.setPosition(0.48);
    }

    public void PickUp0() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.16);
    }

    public void PickUp0Auto() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.48);
    }

    public void PickUp45Right() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.35);
    }

    public void PickUp45Left() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.65);
    }

    public void PickUp90() {
        wristServo.setPosition(0.8);
        rotateServo.setPosition(0.48);
    }

    public void PickUpSpecimen() {}

    public void middleWrist(){
        wristServo.setPosition(0.5);
    }

    public boolean isUp() {
        if (wristServo.getPosition() != 0) {
            return false;
        }
        return true;
    }

    public void moveWristDown(){
        wristServo.setPosition(0.8);
    }

    public void setRotateServo(double pos){
        rotateServo.setPosition(pos);
    }
}