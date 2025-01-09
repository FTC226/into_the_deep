package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subss.Arm;
import org.firstinspires.ftc.teamcode.subss.Slides;

@TeleOp(name="Hanging")
public class hangTest extends OpMode {

    Arm arm = new Arm(this, 0.8);
    Slides slides = new Slides(this);

    @Override
    public void init() {
        arm.init();
        slides.init();
    }

    @Override
    public void loop() {
        if(gamepad1.a)
            slides.hangExtend();
        else if(gamepad1.b)
            arm.moveUp();
        else if(gamepad1.x)
            slides.moveToResetPos();
        else if(gamepad1.y)
            arm.moveDown();
    }
}
