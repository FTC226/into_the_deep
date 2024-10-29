package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "LinearSlideArm")
public class LinearSlideArm extends OpMode {

    public DcMotor arm;

    @Override
    public void init() {
        // Initialize the motor from the hardware map
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set the motor direction if necessary (e.g., DcMotor.Direction.REVERSE)
        arm.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Check if gamepad A is pressed
        if (gamepad1.a) {
            arm.setPower(1.0);  // Run at 100% power
        }
        // Check if gamepad B is pressed
        else if (gamepad1.b) {
            arm.setPower(0.5);  // Run at 50% power
        } else if(gamepad1.x){
            arm.setPower(0.25);
        }
        else if(gamepad1.dpad_down){
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(gamepad1.dpad_up){
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        // If neither button is pressed, stop the motor
        else {
            arm.setPower(0.0);
        }
    }
}