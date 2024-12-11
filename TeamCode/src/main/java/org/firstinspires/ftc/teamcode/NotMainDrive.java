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
@TeleOp (name = "Not Main Drive")
public class NotMainDrive extends LinearOpMode {

    Claw claw;
    Slides slides;
    Arm arm;

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
        waitForStart();
        while(opModeIsActive()){

            if(gamepad2.a){
                new RunClaw(claw);

            }

            if(gamepad2.b){
                arm.moveUp();
                slides.moveUp();
            }
            if(gamepad2.dpad_up){
                //claw.move(true);
                /*
                leftPos +=0.05;
                rightPos -=0.05;
                claw.left.setPosition(leftPos);
                claw.right.setPosition(rightPos);

                 */
                /*
                Actions.runBlocking(

                        new SequentialAction(
                                claw.moveUp1()
                        )
                );
                 */
                new RunClaw(claw);

            }
            if(gamepad2.dpad_down){
                //claw.move(false);

                /*
                    leftPos -=0.05;
                    rightPos +=0.05;
                    claw.left.setPosition(leftPos);
                    claw.right.setPosition(rightPos);
                 */

                /*
                Actions.runBlocking(

                        new SequentialAction(
                                claw.moveDown1()
                        )
                );
                 */
                new RunClaw(claw);
            }

            if(gamepad2.right_stick_y > 0.1 && slidePosition < 2200){
                slidePower = gamepad2.right_stick_y;
                slidePosition += 1;
                slides.move(slidePosition, 1);
            }if(gamepad2.right_stick_y < -0.1 && slidePosition > 0){
                slidePower = gamepad2.right_stick_y;
                slidePosition -= 1;
                slides.move(slidePosition, 1);
            }

            if(gamepad2.left_stick_y > 0.1 && armPosition < 1700){
                armPower = gamepad2.right_stick_y;
                armPosition += 1;
                arm.move(armPosition, 1);
            }if(gamepad2.left_stick_y < -0.1 && armPosition > 0){
                armPower = gamepad2.right_stick_y;
                armPosition -= 1;
                arm.move(armPosition, 1);
            }
            if(gamepad2.dpad_right){
                //claw.rotate(true);
                new RunClaw(claw);
            }
            if(gamepad2.dpad_left){
                //claw.rotate(false);
                new RunClaw(claw);
            }

        }






    }


}
