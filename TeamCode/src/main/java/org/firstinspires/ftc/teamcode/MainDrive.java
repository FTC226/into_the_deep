package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




/** Configuration Notes: CenterStage
 * Port 00: frontLeft
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 */

//@Disabled
@TeleOp (name = "Main Drive")
public class MainDrive extends LinearOpMode {

    Claw claw;
    Slides slides;
    Arm arm;
    SequentialAction runningCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        slides = new Slides(hardwareMap);

        double leftPos = 0, rightPos = 0;
        int slidePosition = 5;
        double slidePower;

        int armPosition = 0;
        double armPower;

        //slides = new Slides(hardwareMap);
        //arm = new Arm(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); //temporary until Drive (Isabel) is developed

        TrajectoryActionBuilder wait = drive.actionBuilder(new Pose2d(0, 0, 0))
                .waitSeconds(1.0);


        waitForStart();
        while (opModeIsActive()) {

            runningCommand = new SequentialAction(claw.open(),claw.close());
            Actions.runBlocking(runningCommand);


        }

/*
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
