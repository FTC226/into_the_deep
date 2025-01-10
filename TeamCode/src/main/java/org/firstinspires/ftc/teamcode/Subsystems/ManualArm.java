package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ManualArm {

    public DcMotorEx arm;
    public int targetPosition;
    Telemetry Telem;
    private ElapsedTime timer = new ElapsedTime();

    public ManualArm(HardwareMap hw, Telemetry tm){
        arm = hw.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPosition = arm.getCurrentPosition();
        Telem = tm;
    }

    public void moveArm(double position){
        targetPosition = targetPosition - ((int)position*5);
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.75);
        Telem.addData("Arm", arm.getCurrentPosition());
        Telem.update();
    }

    public void hold(){
        targetPosition = arm.getCurrentPosition();
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
}
