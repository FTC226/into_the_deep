package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Straight Line Test")
public class straightTest extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public double frontLeftScalar; //set values based on Encoder Test
    public double frontRightScalar; //set values based on Encoder Test
    public double backLeftScalar; //set values based on Encoder Test
    public double backRightScalar; //set values based on Encoder Test

    public void init(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    public void loop(){
        double power = -gamepad1.left_stick_y; //get power from the joystick

        //apply power to each wheel, with the adjusted values
        frontLeft.setPower(frontLeftScalar*power);
        frontRight.setPower(frontRightScalar*power);
        backLeft.setPower(backLeftScalar*power);
        backRight.setPower(backRightScalar*power);
    }
}
