package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Claw;

@TeleOp(name = "Wrist Tester")
public class ClawCameraTest extends OpMode {
    Camera camera = new Camera(this);
    Claw claw = new Claw(this);

    @Override
    public void init() {
        claw.init();
        camera.init();
    }
    @Override
    public void loop() {
//        telemetry.addData("Angle", camera.());
//        telemetry.addData("Center X", camera.());
//        telemetry.addData("Center Y", camera.realY());
        telemetry.update();
    }
}