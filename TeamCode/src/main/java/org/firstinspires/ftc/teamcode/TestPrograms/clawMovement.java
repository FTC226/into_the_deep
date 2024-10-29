package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Claw Mover Tester")
public class clawMovement extends OpMode {
    public CRServo left, right, claw;
    public ElapsedTime runtime;
    public double armPower;
    public double wristPower;
    public double stop = 0;
    public void init(){
        left = hardwareMap.get(CRServo.class, "leftServo");
        right = hardwareMap.get(CRServo.class, "rightServo");
        claw = hardwareMap.get(CRServo.class, "clawServo");
        runtime = new ElapsedTime();

        runtime.reset();

        /*
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
        */
    }

    public void loop(){
        runtime.reset();

        armPower = -gamepad2.left_stick_y;
        wristPower = gamepad2.left_stick_x;

        if(armPower > 0.5 && stop < 500){
            left.setPower(-armPower);
            right.setPower(armPower);
            stop++;
        }

        else if(armPower < -0.5 && stop > 0){

            left.setPower(-armPower);
            right.setPower(armPower);
            stop--;
        }

        else if(wristPower > 0.5){
            left.setPower(armPower);
            right.setPower(armPower);

        }
        else if(wristPower < -0.5){
            left.setPower(-armPower);
            right.setPower(-armPower);
        } else{
            left.setPower(0);
            right.setPower(0);
        }

    }
}
