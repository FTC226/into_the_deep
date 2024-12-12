package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    Claw claw;
    Slides slides;
    Arm arm;
    Action clawCommand, armCommand, slideCommand;
    SequentialAction runningAction;
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

            clawCommand = (gamepad2.a) ? claw.open() : claw.close();
            armCommand = (gamepad2.b) ? arm.moveUp() : arm.hold();
            slideCommand = (gamepad2.x) ? slides.moveUp() : slides.hold();

            runningAction = (gamepad2.y) ?
                    new SequentialAction(arm.moveUp(), Wait(), Wait(), Wait(), Wait(), slides.moveUp()) :
                    new SequentialAction(arm.hold(),slides.hold());
            Actions.runBlocking(slideCommand);



        }

/*

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
