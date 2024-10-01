package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Pathing")
public class autoPathing extends OpMode {

    FieldCentric Robot = new FieldCentric();
    double leftStickY;
    double leftStickX;
    double rightStickX;
    ElapsedTime runtime = new ElapsedTime();
    boolean change;

    public void init(){
        Robot.init(hardwareMap);
        leftStickY = 0.0; leftStickX = 0.0; rightStickX = 0.0; //change = false;//Spline path

        Robot.fieldCentric(0.0,0.0,0.0,true);
    }

    public void start(){
        runtime.reset();
    }

    public void loop(){
        Robot.fieldCentric(leftStickY, +leftStickX, rightStickX, false);
        telemetry.addData("Time", runtime.milliseconds());

        moveToSpline();
    }

    public void moveToSpline(){
        if (runtime.milliseconds()<2500){//t>0.5 seconds - bot moves forward
            leftStickY=1.0;
            leftStickX=0.0;
        } else if(runtime.milliseconds()<5000){//0.5<t<1.5 seconds - bot changes to move up right
            leftStickY=0.5;
            leftStickX=0.5;
        } else if(runtime.milliseconds()<7500){//1.5<t<2.0 seconds - bot changes to move forward
            leftStickY=1.0;
            leftStickX=0;
        } else {
            leftStickY = 0.0;
            leftStickX = 0.0;
            rightStickX = 0.0;
        }
    }

    public void moveTo(){
        if(runtime.milliseconds()<5000){
            rightStickX=1.0;
            leftStickX=0.5;
            leftStickY=0.5;
        } else {
            rightStickX = 0.0;
            leftStickX = 0.0;
            leftStickY = 0.0;
        }
    }

}
