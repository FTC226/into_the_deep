package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    OpMode opMode;

    public Claw(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        claw = opMode.hardwareMap.get(Servo.class, "clawServo");
    }

    public void closeClaw() {
        claw.setPosition(1);
    }
    public void openClaw() {
        claw.setPosition(0);
    }
}
