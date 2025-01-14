package org.firstinspires.ftc.teamcode.test.tester;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@Config
@TeleOp(name = "Tester - Wrist")
public class TesterWrist extends OpMode {
    public Servo left, right, claw;
    public ElapsedTime runtime;

    public static double leftPos, rightPos;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){
        left = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        leftPos = 0.256;
        rightPos = 0.636;

        left.setPosition(leftPos);
        right.setPosition(rightPos);
    }

    @Override
    public void loop(){
        if (-gamepad1.left_stick_y > 0.1) {
            leftPos += 0.002;
        } else if (-gamepad1.left_stick_y < -0.1) {
            leftPos -= 0.002;
        }

        if (-gamepad1.right_stick_y > 0.1) {
            rightPos += 0.002;
        } else if (-gamepad1.right_stick_y < -0.1) {
            rightPos -= 0.002;
        }

        if (gamepad1.a) {
            leftPos = 0.1; //bottom
            rightPos = 0.1;
        } else if (gamepad1.b) {
            leftPos = 0.3; //top
            rightPos = 0.3;
        } else if (gamepad1.x) {
            leftPos = 0.48;
            rightPos = 0.48; //bottom 45
        } else if (gamepad1.y) {
            leftPos = 0.9; //bottom 90
            rightPos = 0.9;
        }

        if (leftPos < -1) {
            leftPos = -1;
        } else if (leftPos > 1) {
            leftPos = 1;
        }

        if (rightPos < -1) {
            rightPos = -1;
        } else if (rightPos > 1) {
            rightPos = 1;
        }

        if(gamepad1.dpad_up) {
            left.setPosition(leftPos);
            right.setPosition(rightPos);
        }

        if(gamepad1.right_bumper) {
            claw.setPosition(0);
        }
        if(gamepad1.left_bumper) {
            claw.setPosition(1);
        }



        telemetry.addData("Left pos ", leftPos);
        telemetry.addData("Right pos ", rightPos);
        telemetry.update();
    }
}