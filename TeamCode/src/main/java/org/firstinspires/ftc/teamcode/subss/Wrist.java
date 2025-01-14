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
        leftClaw.setPosition(-1);
        rightClaw.setPosition(0.666);
    }

    public void placeSample() {
        leftClaw.setPosition(0.102);
        rightClaw.setPosition(0.614);
    }

    public void Down() {
        leftClaw.setPosition(0.316);
        rightClaw.setPosition(0.366);
    }
    public void PickUp0() {
        leftClaw.setPosition(0.316);
        rightClaw.setPosition(0.366);
    }

    public void PickUp45() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
    }

    public void PickUp90() {
        leftClaw.setPosition(0.168);
        rightClaw.setPosition(0.2);
    }
    public void PlaceSpecimen() {
        leftClaw.setPosition(0.088);
        rightClaw.setPosition(0.652);
    }

    public void PickUpSpecimen() {
        leftClaw.setPosition(0.182);
        rightClaw.setPosition(0.522);
    }

    public void PickUpSpecimenGround() {
        leftClaw.setPosition(0.239);
        rightClaw.setPosition(0.46);
    }

    public boolean isUp() {
        if (leftClaw.getPosition() != -1 && rightClaw.getPosition() != 0.666) {
            return false;
        }
        return true;
    }
}
