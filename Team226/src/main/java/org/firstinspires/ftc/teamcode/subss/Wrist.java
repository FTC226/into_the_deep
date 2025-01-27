package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Wrist {
    Servo wristServo;
    Servo rotateServo;
    OpMode opMode;

    public Wrist(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        wristServo = opMode.hardwareMap.get(Servo.class, "wristServo"); //leftServo
        rotateServo = opMode.hardwareMap.get(Servo.class, "rotateServo"); //rightServo
    }

    public void Up() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void Down() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PlaceSample() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PickUp0() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PickUp45Right() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PickUp45Left() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PickUp90() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PlaceSpecimen() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }

    public void PickUpSpecimen() {
        wristServo.setPosition(0.408);
        rotateServo.setPosition(0.544);
    }


    public boolean isUp() {
        if (wristServo.getPosition() != 0.408) {
            return false;
        }
        return true;
    }
}
