package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "PIDFLinearSlide")
public class PIDFLinearSlide extends LinearOpMode {

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    public static final double p = 0.0, i = 0.0, d = 0.0, f = 0.0;

    public int targetPosition = 0;

    public PIDFController pid = new PIDFController(p, i, d, f);
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while(opModeIsActive()) {

            targetPosition = (int) -gamepad2.left_stick_y;

            double jsY = -gamepad1.left_stick_y;
            double leftEncoder = leftSlide.getCurrentPosition();
            double rightEncoder = rightSlide.getCurrentPosition();

            double power = pid.calculate(leftEncoder, targetPosition);

            if (jsY > 0) {
                targetPosition += 10;

                leftSlide.setPower(power);
                rightSlide.setPower(power);
            } else if (jsY < 0 && targetPosition != 0) {
                targetPosition -= 10;

                leftSlide.setPower(-power);
                rightSlide.setPower(-power);
            } else if (leftEncoder >= 800 || rightEncoder >= 800) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }

            packet.put("p: ", p);
            packet.put("i: ", i);
            packet.put("d: ", d);
            packet.put("f: ", f);
            packet.put("targetPosition: ", targetPosition);
            packet.put("JoyStick: ", jsY);


//            if(power<0 && gamepad2.a){
//                leftSlide.setPower(-1.0);
//                rightSlide.setPower(-1.0);
//            } else if (power < 0){
//                leftSlide.setPower(-0.01);
//                rightSlide.setPower(-0.01);
//            } else{
//                leftSlide.setPower(power);
//                rightSlide.setPower(power);
//            }
        }

        dashboard.sendTelemetryPacket(packet);

    }
}
