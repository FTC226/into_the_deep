package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class linearSlides {
    DcMotor motor;
    HardwareMap hwMap;
    public static final double upperLimit = 2900;

    public void init(HardwareMap awMap, String name){
        this.hwMap = awMap;

        motor = hwMap.get(DcMotor.class, name);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getPosition(){
        return(motor.getCurrentPosition());
    }

    public void move(double power){
        if(power>0.1 && getPosition()<upperLimit){ //go up on joystick/linear slide
            motor.setPower(power);

        } else if(power<-0.1 && getPosition()>5){ //go down on joystick/linear slide
            motor.setPower(-0.5);

        } else{
            motor.setPower(0.05);
        }
    }
}
