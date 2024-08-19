package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="Motor Test")
public class motorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // tests the direction of each wheel
            // y: frontLeft
            // b: frontRight
            // a: backRight
            // x: backLeft
            if (gamepad1.y) {
                frontLeft.setPower(0.5);
            } else if (gamepad1.b) {
                frontRight.setPower(0.5);
            } else if (gamepad1.a) {
                backRight.setPower(0.5);
            } else if (gamepad1.x) {
                backLeft.setPower(0.5);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }
}