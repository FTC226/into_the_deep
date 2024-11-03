package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp (name = "armPID")
public class armPID extends OpMode {

    private PIDController controller;

    public static double kp = 0.05, ki = 0.0, kd = 0.0;
    public static double kf = 0.1;

    public static int target = 0;

    private final double ticks_in_degree = 700 / 180.0;

    private DcMotorEx arm;

    @Override
    public void init() {
        controller = new PIDController(kp, ki, kd);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }


    @Override
    public void loop() {
        controller.setPID(kp, ki, kd);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * kf;

        double power = pid + ff;

        arm.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

}