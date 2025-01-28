package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ManualArm;
import org.firstinspires.ftc.teamcode.Subsystems.ManualClaw;
import org.firstinspires.ftc.teamcode.Subsystems.ManualSlides;

@TeleOp (name = "Michael Back Up")
public class MichaelBackUp extends LinearOpMode {
    public ManualArm arm;
    public ManualClaw claw;
    public ManualSlides slides;
    public Drive drive;
    Telemetry Telem;
    @Override
    public void runOpMode() throws InterruptedException{
        drive = new Drive(hardwareMap, telemetry);
        arm = new ManualArm(hardwareMap,telemetry);
        slides = new ManualSlides(hardwareMap,telemetry);
        claw = new ManualClaw(hardwareMap);

        drive.driveFC(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.x);

        if(gamepad2.dpad_left){
            claw.turn(false);
        }
        if(gamepad2.dpad_right){
            claw.turn(true);
        }

        if(gamepad2.dpad_up){
            claw.moveClaw(true);
        }
        if(gamepad2.dpad_down){
            claw.moveClaw(false);
        }

        if(gamepad2.right_bumper){
            claw.open();
        } else{
            claw.close();
        }

        if(gamepad2.left_stick_y >0.05 || gamepad2.left_stick_y < -0.05){
            arm.moveArm(gamepad2.left_stick_y);
        } else{
            arm.hold();
        }

        if(gamepad2.right_stick_y >0.05 || gamepad2.right_stick_y < -0.05){
            slides.moveSlides(gamepad2.right_stick_y);
        } else{
            arm.hold();
        }
    }
}
