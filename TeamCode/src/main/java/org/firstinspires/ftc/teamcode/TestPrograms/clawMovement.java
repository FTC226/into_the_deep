package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Claw Mover Tester")
public class clawMovement extends OpMode {
    public Servo left, right;
    public ElapsedTime runtime;

    public double angle = 0.0;


    public void init(){
        left = hardwareMap.get(Servo.class, "leftServo");
        right = hardwareMap.get(Servo.class, "rightServo");
        runtime = new ElapsedTime();

        runtime.reset();

        while(runtime.milliseconds()<1000){
            left.setPosition(0.0);
            right.setPosition(0.0);
        }

        while(runtime.milliseconds()<5000){
            left.setPosition(1.0);
            right.setPosition(1.0);
        }

        while(runtime.milliseconds()<10000){
            left.setPosition(-1.0);
            right.setPosition(-1.0);
        }

        left.setPosition(0.0);
        right.setPosition(0.0);

    }

    public void loop(){
        runtime.reset();

        double leftPower = -gamepad2.left_stick_y;
        double rightPower = -gamepad2.right_stick_y;

        while (angle >= 92.0 || angle <= 88.0) {
            left.setPosition(-1.0);
            right.setPosition(1.0);
        }


        left.setPosition(leftPower);
        right.setPosition(rightPower);
    }
}
