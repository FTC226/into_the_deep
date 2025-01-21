package org.firstinspires.ftc.teamcode.subss;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Wrist {
    Servo leftClaw;
    Servo rightClaw;
    OpMode opMode;

    public Wrist(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        leftClaw = opMode.hardwareMap.get(Servo.class, "leftServo");
        rightClaw = opMode.hardwareMap.get(Servo.class, "rightServo");
    }

    public void Up() {
        leftClaw.setPosition(0.408);
        rightClaw.setPosition(0.544);
    }

    public void placeSample() {
        leftClaw.setPosition(0.468);
        rightClaw.setPosition(0.494);
    }

    public void Down() {
        leftClaw.setPosition(0.728);
        rightClaw.setPosition(0.222);
    }
    public void PickUp0() {
        leftClaw.setPosition(0.728);
        rightClaw.setPosition(0.222);
    }

    public void PickUp90() {
        leftClaw.setPosition(0.91);
        rightClaw.setPosition(0.382);
    }
    public void PlaceSpecimen() {
        leftClaw.setPosition(0.408);
        rightClaw.setPosition(0.544);
    }

    public void PlaceSpecimenAuto() {
        leftClaw.setPosition(0.45);
        rightClaw.setPosition(0.528);
    }

    public void PickUpSpecimen() {
        leftClaw.setPosition(0.57);
        rightClaw.setPosition(0.368);
    }
    public void PickUpSpecimenOp() {
        leftClaw.setPosition(0.57);
        rightClaw.setPosition(0.368);
    }

    public void PickUpSpecimenGround() {
        leftClaw.setPosition(0.239);
        rightClaw.setPosition(0.46);
    }

    public boolean isUp() {
        if (leftClaw.getPosition() != 0.408 && rightClaw.getPosition() != 0.544) {
            return false;
        }
        return true;
    }
}
