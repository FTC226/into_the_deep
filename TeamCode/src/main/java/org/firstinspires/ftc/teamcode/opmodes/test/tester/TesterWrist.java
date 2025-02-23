package org.firstinspires.ftc.teamcode.opmodes.test.tester;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subss.Arm;
import org.firstinspires.ftc.teamcode.subss.Slides;

//@Disabled
@Config
@TeleOp(name = "Tester - Wrist")
public class TesterWrist extends OpMode {
    Arm arm = new Arm(this, 1);
    Slides slides = new Slides(this);


    public Servo claw, wrist, rotate;
    public ElapsedTime runtime;


    public static double  wristPos, rotatePos, clawPos;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){

        arm.init();
        slides.init();

        claw = hardwareMap.get(Servo.class, "clawServo");
        rotate = hardwareMap.get(Servo.class, "rotateServo");
        wrist = hardwareMap.get(Servo.class, "wristServo");


        claw.setPosition(0);
        rotate.setPosition(0.48);
        wrist.setPosition(0.2);

    }

    @Override
    public void loop(){

        if (gamepad1.b) {
            wrist.setPosition(wristPos);
        }
        if (gamepad1.x) {
            rotate.setPosition(rotatePos);
        }

        if(gamepad1.right_bumper) {
            claw.setPosition(0);
        }
        if(gamepad1.left_bumper) {
            claw.setPosition(clawPos);
        }

        telemetry.addData("wristPos ", wristPos);
        telemetry.addData("rotatePos ", rotatePos);
        telemetry.update();
    }
}