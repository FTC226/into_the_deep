package org.firstinspires.ftc.teamcode.subsystems.SubsystemsTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends DriveParent {
    public DcMotorEx arm;
    public int targetPosition;
    HardwareMap HwMap;
    Telemetry Telem;
    private ElapsedTime timer = new ElapsedTime();


    public Arm(HardwareMap hw, Telemetry tm){
        super(hw, tm);
        this.HwMap = hw;
        arm = HwMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Telem = tm;

    }

    public class MoveUp implements Action {
        private boolean timerStarted = false;
        double x,y,rx;
        boolean resetIMU;
        public MoveUp(double ax, double ay, double arx, boolean aresetIMU){
            this.x = ax;
            this.y = ay;
            this.rx = arx;
            this.resetIMU = aresetIMU;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 1570;
            arm.setTargetPosition(1570);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.3);
            Telem.addData("Arm", arm.getCurrentPosition());
            Telem.update();
            drive(x,y,rx,resetIMU);
//            if(Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())<5){
//                timer.reset();
//                timerStarted = true;
//            }

            return Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())>10; // || !timerStarted || timer.seconds()<0.5
        }

    }

    public class MoveUpSpecimen implements Action {
        private boolean timerStarted = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 700;
            arm.setTargetPosition(700);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.3);
            Telem.addData("Arm", arm.getCurrentPosition());
            Telem.update();
//            if(Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())<5){
//                timer.reset();
//                timerStarted = true;
//            }

            return Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())>10; // || !timerStarted || timer.seconds()<0.5
        }

    }

    public Action moveUpSpecimen(){return new MoveUpSpecimen();}

    public Action moveUp(double x, double y, double rx, boolean resetIMU){
        return new MoveUp(x,y,rx,resetIMU);
    }

    public Action moveUp(){
        return null;
    }

    public class Hold implements Action {
        double x,y,rx;
        boolean resetIMU;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            arm.setTargetPosition(targetPosition);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            return false;
        }

    }

    public Action hold(){
        return new Hold();
    }

    public class MoveDown implements  Action{

        double x,y,rx;
        boolean resetIMU;
        public MoveDown(double ax, double ay, double arx, boolean aresetIMU){
            this.x = ax;
            this.y = ay;
            this.rx = arx;
            this.resetIMU = aresetIMU;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            targetPosition = 0;
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            Telem.addData("Arm", arm.getCurrentPosition());
            Telem.update();
            drive(x,y,rx,resetIMU);
            return Math.abs(arm.getTargetPosition()-arm.getCurrentPosition())>10;
        }
    }

    public Action moveDown(double x, double y, double rx, boolean resetIMU){
        return new MoveDown(x,y,rx,resetIMU);
    }

    public Action moveDown(){
        return null;
    }


    public void move(int position, double power){

        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }




}