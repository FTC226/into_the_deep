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
        leftClaw.setPosition(0.15);
        rightClaw.setPosition(0.684);
    }

    public void Down() {
        leftClaw.setPosition(0.6);
        rightClaw.setPosition(0.4);
    }
    public void PickUp0() {
        leftClaw.setPosition(0.6);
        rightClaw.setPosition(0.4);
    }

    public void PickUp45() {
        leftClaw.setPosition(0.7);
        rightClaw.setPosition(0.4);
    }

    public void PickUp90() {
        leftClaw.setPosition(0.9);
        rightClaw.setPosition(0.55);
    }
    public void PlaceSpecimen() {
        leftClaw.setPosition(0.3);
        rightClaw.setPosition(0.65);
    }

    public void PickUpSpecimen() {
        leftClaw.setPosition(0.45);
        rightClaw.setPosition(0.55);
    }

    public void PickUpSpecimenGround() {
        leftClaw.setPosition(0.48);
        rightClaw.setPosition(0.48);
    }

    public boolean isUp() {
        if (leftClaw.getPosition() != 0.15 && rightClaw.getPosition() != 0.684) {
            return false;
        }
        return true;
    }
}
