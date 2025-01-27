package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Forearm {
    Servo leftForearmServo;
    Servo rightForearmServo;
    OpMode opMode;

    public Forearm(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftForearmServo = opMode.hardwareMap.get(Servo.class, "leftForearmServo"); //leftServo
        rightForearmServo = opMode.hardwareMap.get(Servo.class, "rightForearmServo"); //rightServo
    }

    public void ReadyPickUpSample() {
        leftForearmServo.setPosition(0);
        rightForearmServo.setPosition(0);
    }

    public void PickUpSample() {
        leftForearmServo.setPosition(0);
        rightForearmServo.setPosition(0);
    }

    public void PlaceSample() {
        leftForearmServo.setPosition(0);
        rightForearmServo.setPosition(0);
    }

    public void PickUpSpecimen() {
        leftForearmServo.setPosition(0);
        rightForearmServo.setPosition(0);
    }

    public void ReadyPlaceSpecimen() {
        leftForearmServo.setPosition(0);
        rightForearmServo.setPosition(0);
    }

    public void PlaceSpecimen() {
        leftForearmServo.setPosition(0);
        rightForearmServo.setPosition(0);
    }
}
