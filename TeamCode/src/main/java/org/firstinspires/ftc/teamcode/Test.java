package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="Test")
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // check if the user has stopped the program
        if (isStopRequested()) return;

        // the loop
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;

            frontLeftMotor.setPower(y);
            backLeftMotor.setPower(y);
            frontRightMotor.setPower(y);
            backRightMotor.setPower(y);
        }
    }
}