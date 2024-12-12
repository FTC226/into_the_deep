package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class Robot {
    public Arm arm;
    public Claw claw;
    public Slides slides;
    Telemetry Telem;

    public Robot(HardwareMap hwMap, Telemetry tm){
        arm = new Arm(hwMap, tm);
        claw = new Claw(hwMap);
        slides = new Slides(hwMap, tm);
        Telem = tm;
    }


    public class PlaceSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            claw.moveDown();
            slides.moveDown();

            arm.moveUp();
            //waitMillis(200);//adjust as needed

            slides.moveUp();
            //waitMillis(1500);//adjust as needed

            claw.moveUp();
            //waitMillis(200);//adjust as needed

            claw.open();
            //waitMillis(200);//adjust as needed

            claw.close();
            //waitMillis(200);//adjust as needed

            return false;
        }
    }
    public SequentialAction placeSample(){
        //return new PlaceSample();

        return new SequentialAction(arm.moveUp(),//waitMillis(200);//adjust as needed
                                    slides.moveUp(), //waitMillis(1500);//adjust as needed
                                    claw.moveUp(), //waitMillis(200);//adjust as needed
                                    claw.open(), //waitMillis(200);//adjust as needed
                                    claw.close()
        );
    }


    public class ResetPosition implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            claw.moveDown();
            //waitMillis(200);//adjust time as needed

            slides.moveDown();
            //waitMillis(1500);//adjust time as needed

            arm.moveDown();
            //waitMillis(200);//adjust time as needed

            return false;
        }
    }
    public Action resetPosition(){
        return new ResetPosition();
    }

    public void waitMillis(int time){
        try {
            sleep(time);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /*
    public boolean checkMovement(){
        boolean slideMovement = slides.leftSlide.getVelocity()>5 && slides.rightSlide.getVelocity()>5;
        boolean armMovement = arm.arm.getVelocity()>5;

        return slideMovement || armMovement;
    }*/
}