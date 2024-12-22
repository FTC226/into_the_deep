package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

public class Robot {
    public Arm arm;
    public Claw claw;
    public Slides slides;
    Telemetry Telem;
    private ElapsedTime timer = new ElapsedTime();


    public Robot(HardwareMap hwMap, Telemetry tm){
        arm = new Arm(hwMap, tm);
        claw = new Claw(hwMap);
        slides = new Slides(hwMap, tm);
        Telem = tm;
    }


    public class PlaceSample implements Action {
        private boolean timerStarted = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.moveUp();
            claw.close();

            arm.moveUp();

            if (Math.abs(arm.arm.getTargetPosition()-arm.arm.getCurrentPosition())<5) {
                slides.moveUp();
                //telemetry.addData("moving slides", null);
            }

            if (Math.abs(slides.leftSlide.getTargetPosition()-slides.leftSlide.getCurrentPosition())<5) {
                claw.moveDown();
                timer.reset();
                timerStarted = true;
            }

            if (timerStarted && timer.seconds() > 3) {
                claw.open();
            }

            return timerStarted || timer.seconds()<5;
        }
    }
    public Action placeSample(){
        //return new PlaceSample();

        return new SequentialAction(
                claw.moveDown(),
                arm.moveUp(),//waitMillis(200);//adjust as needed
                slides.moveUp() //waitMillis(1500);//adjust as needed
                //waitMillis(200);//adjust as needed
                //waitMillis(200);//adjust as needed

        );
    }

    public Action moveSub(){
        return new SequentialAction(
                slides.moveDown(),
                claw.moveUp(),
                arm.moveDown(),
                slides.moveSub(),
                claw.moveDown(),
                claw.openPerm()
        );
    }

    public Action score(){
        return new SequentialAction(
                claw.moveUp(),
                claw.open(),
                claw.close()
        );
    }

    public Action holdPosition(){
        return new SequentialAction(
                arm.hold(),
                slides.hold(),
                claw.hold()
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
        return new SequentialAction(
                claw.close(),
                claw.moveMiddle(),
                slides.moveDown(),
                arm.moveDown()//waitMillis(200);//adjust as needed
                //waitMillis(1500);//adjust as needed
                //waitMillis(200);//adjust as needed
                //waitMillis(200);//adjust as needed

        );
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