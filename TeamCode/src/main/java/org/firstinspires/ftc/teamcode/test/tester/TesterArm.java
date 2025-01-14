package org.firstinspires.ftc.teamcode.test.tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name="Tester - Arm")
public class TesterArm extends OpMode {
    DcMotorEx arm;
    public void init(){
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){
        if (gamepad1.a) {
            arm.setTargetPosition(1600);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(1600);
        } else if(gamepad1.b){
            arm.setTargetPosition(10);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(1600);
        }

        if(Math.abs(arm.getCurrentPosition())<5)
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Arm", arm.getCurrentPosition());
        telemetry.update();

    }


}
