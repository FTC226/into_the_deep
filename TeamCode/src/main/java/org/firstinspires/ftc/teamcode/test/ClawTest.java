package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "ClawTester")
public class ClawTest extends OpMode {

    CRServo leftServo, rightServo;
    Servo clawServo, clawTest;


    @Override
    public void init() {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawTest = hardwareMap.get(Servo.class, "clawTest");


    }
    public void loop() {

        if (gamepad2.dpad_up) {
            clawServo.setPosition(0);
        } else {
            clawServo.setPosition(1);

        }

        if (gamepad2.left_stick_y > 0.1) {
            leftServo.setPower(gamepad2.left_stick_y);
        }
        if (gamepad2.right_stick_y > 0.1) {
            rightServo.setPower(gamepad2.right_stick_y);
        }

        if (gamepad2.dpad_down) {
            clawTest.setPosition(0);
        } else {
            clawTest.setPosition(1);
        }

        telemetry.addData("left:", gamepad2.left_stick_y);
        telemetry.addData("right:", gamepad2.right_stick_y);
        telemetry.addData("dpad up:", gamepad2.dpad_up);
        telemetry.addData("dpad down:", gamepad2.dpad_down);

    }
}
