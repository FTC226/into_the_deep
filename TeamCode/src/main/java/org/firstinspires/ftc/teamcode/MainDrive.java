package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/** Configuration Notes: CenterStage
 * Port 00: frontLeft
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 */

//@Disabled
@TeleOp (name = "Main Drive")
public class MainDrive extends LinearOpMode {

    Robot robot;
    Action clawCommand, armCommand, slideCommand;
    Action RunningCommand;
    ElapsedTime runtime = new ElapsedTime();

    public class Wait implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            runtime.reset();
            while(runtime.seconds()<1){}
            return false;
        }
    }

    public Action Wait() {
        return new Wait();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            //RunningCommand = (gamepad2.a) ? robot.placeSample(): robot.holdPosition();
            //RunningCommand = (gamepad2.b) ? robot.resetPosition(): robot.holdPosition();


            if(gamepad2.a) //move up sample
                RunningCommand = robot.placeSample();
            else if(gamepad2.b) //move down to drive
                RunningCommand = robot.resetPosition();
            else if(gamepad2.x)
                RunningCommand = robot.moveSub();
            else if(gamepad2.y)
                RunningCommand = robot.score();
            else
                RunningCommand = robot.holdPosition();



            robot.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.x);

            Actions.runBlocking(
                    RunningCommand

            );



        }

/*

clawCommand = (gamepad2.a) ? claw.open() : claw.close();
            armCommand = (gamepad2.b) ? arm.moveUp() : arm.hold();
            slideCommand = (gamepad2.x) ? slides.moveUp() : slides.hold();

            runningAction = (gamepad2.y) ?
                    new SequentialAction(arm.moveUp(), Wait(), Wait(), Wait(), Wait(), slides.moveUp()) :
                    new SequentialAction(arm.hold(),slides.hold());
if(gamepad2.b)
                armCommand = arm.moveUp();
            else
                armCommand = arm.hold();


    if(gamepad2.a){
                runningCommand = new SequentialAction(claw.open());
            } else{
                runningCommand = new SequentialAction(claw.close());
            }

            if(gamepad2.b){
                Actions.runBlocking(

                        new SequentialAction(
                                arm.moveUp(),
                                wait.build(),
                                slides.moveUp(),
                                wait.build()


                        )
                );
            }
            if(gamepad2.dpad_up){
                claw.moveUp();

        //new RunClaw(claw);

    }
            if(gamepad2.dpad_down){
        claw.moveDown();


    }

            if(gamepad2.right_stick_y > 0.1 && slidePosition < 2200){
        slidePower = gamepad2.right_stick_y;
        slidePosition += 1;
        slides.move(slidePosition, 1);
    }else if(gamepad2.right_stick_y < -0.1 && slidePosition > 0){
        slidePower = gamepad2.right_stick_y;
        slidePosition -= 1;
        slides.move(slidePosition, 1);
    } else{
        slides.move(slidePosition, 0);
    }

            if(gamepad2.left_stick_y > 0.1 && armPosition < 1700){
        armPower = gamepad2.right_stick_y;
        armPosition += 1;
        arm.move(armPosition, 1);
    } else if(gamepad2.left_stick_y < -0.1 && armPosition > 0){
        armPower = gamepad2.right_stick_y;
        armPosition -= 1;
        arm.move(armPosition, 1);
    }else{
        arm.move(armPosition, 0);
    }
            if(gamepad2.dpad_right){
        claw.rotate(true);
        //new RunClaw(claw);
    }
            if(gamepad2.dpad_left){
        claw.rotate(false);
        //new RunClaw(claw);
    }

}

 */
    }
}