package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Wrist Tester")
public class WristTester extends OpMode {
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
            leftPos = 0.256;
            rightPos = 0.636;
        } else if (gamepad1.b) {
            leftPos = 0.612;
            rightPos = 0.266;
        } else if (gamepad1.x) {
            leftPos = 0.524;
            rightPos = 0.35;
        } else if (gamepad1.y) {
            leftPos = 0.482;
            rightPos = 0.406;
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

        left.setPosition(leftPos);
        right.setPosition(rightPos);

        telemetry.addData("Left pos ", leftPos);
        telemetry.addData("Right pos ", rightPos);
        telemetry.update();
    }
}