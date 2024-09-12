package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/** Configuration Notes: CenterStage
 * Port 00: frontLeft
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 */

//@Disabled
@TeleOp (name = "TeleOp - Test")
public class Drive_07_31 extends OpMode {

    CougarRobot Robot = new CougarRobot();

    @Override
    public void init(){

        //Creating Robot Object
        Robot.init(hardwareMap);

    }

    @Override
    public void loop(){
        //Gathering Movement Control Data:
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX =  -gamepad1.left_stick_x * 1.5; //multiply to account for strafing
        double rightStickX = gamepad1.right_stick_x;

        //Using movement control data:
        if(leftStickY > 0){
            telemetry.addData("Moving","Forwards");
        }else if(leftStickY < 0){
            telemetry.addData("Moving","Backwards");
        }
        else{
            telemetry.addData("Moving", "Not Moving");
        }

        //Updating telemetry:
        telemetry.addData("Top Left Power ", leftStickY + leftStickX + rightStickX);
        telemetry.addData("Bottom Left Power ", leftStickY + leftStickX + rightStickX);
        telemetry.addData("Top Right Power ", leftStickY + leftStickX - rightStickX);
        telemetry.addData("Bottom Right Power ", leftStickY + leftStickX - rightStickX);

        //Moving robot:
        Robot.moveRobot(leftStickY,leftStickX,rightStickX);
    }
}
