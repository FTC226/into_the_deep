package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.ftccommon.configuration.EditLynxModuleActivity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Field Centric PID")
//@Config
@Disabled

public class FieldCentricPID extends OpMode {

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;

    // Gamepad
    public double leftStickY, leftStickX, rightStickX;

    // PID
    public static double Kp = 0.5;
    public static double Ki = 0;
    public static double Kd = 0.1;

    public double targetYaw;
    double integralSum = 0;
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    // FTC Dashboard

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void loop() {
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;

        robotOrientation = imu.getRobotYawPitchRollAngles();

        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);
        robotYaw = (robotYaw + Math.PI) % (2 * Math.PI) - Math.PI;

        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        //  || Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate) > 1
        if (Math.abs(rightStickX) > 0.1) {
            targetYaw = robotYaw;
            lastError = 0;
        }

        if (gamepad1.x) {
            imu.resetYaw();
            targetYaw = 0;
            integralSum = 0;
            lastError = 0;
        }

        // PID Calculations
        double error = targetYaw - robotYaw;
        error = (error + Math.PI) % (2 * Math.PI) - Math.PI;

        if (Math.abs(error) < Math.toRadians(2)) { // 2° tolerance
            error = 0;
        }

        // Compute PID Terms
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double correction = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        double rotationPower = Math.abs(rightStickX) > 0.1 ? rightStickX : -correction;

        frontLeft.setPower((rotY + rotX + rotationPower) / denominator);
        frontRight.setPower((rotY - rotX - rotationPower) / denominator);
        backLeft.setPower((rotY - rotX + rotationPower) / denominator);
        backRight.setPower((rotY + rotX - rotationPower) / denominator);

        lastError = error;
        timer.reset();

        // FTC Dashboard
        telemetry.addData("Target: ", Math.toDegrees(targetYaw));
        telemetry.addData("Actual: ", Math.toDegrees(robotYaw));
        telemetry.addData("Error: ", Math.toDegrees(error));
        telemetry.addData("Yaw Acceleration", Math.abs(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
        telemetry.update();
    }
}