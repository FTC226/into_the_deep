package org.firstinspires.ftc.teamcode.TestPrograms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp (name = "Linear And Arm")
public class LinearAndArm extends OpMode {

    public DcMotorEx arm;
    public DcMotorEx leftSlide, rightSlide;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    public long previousTime = 0;
    public double ePrevious = 0;
    public double eIntegral = 0;

    public static int target = 0;


    public static double p = 0.03, i = 0.3, d = 0.0002;

    private PIDController controller;

    public static double kp = 0.05, ki = 0.0, kd = 0.0;
    public static double kf = 0.1;

    public static int targetArm = 0;

    private final double ticks_in_degree = 700 / 180.0;


    @Override
    public void init() {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide = hardwareMap.get(DcMotorEx .class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(kp, ki, kd);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {

        dashboard.sendTelemetryPacket(packet);

        double jS = -gamepad2.left_stick_y;

        int leftEncoder = leftSlide.getCurrentPosition();
        int rightEncoder = rightSlide.getCurrentPosition();

        double power = pidController(leftEncoder, p,i,d);

        leftSlide.setPower(jS);
        rightSlide.setPower(jS);
//        moveMotor(power);



//        if (rightEncoder > leftEncoder) {
//            leftSlide.setPower(0);
//            rightSlide.setPower(0);
//        } else {
//        }


        packet.put("p", p);
        packet.put("i", i);
        packet.put("d", d);
        packet.put("kp", kp);
        packet.put("ki", ki);
        packet.put("kd", kd);
        packet.put("kf", kf);
        packet.put("Left Motor:", Math.abs(leftEncoder));
        packet.put("Right Motor:", Math.abs(rightEncoder));
        packet.put("Left-Right Motor:", Math.abs(leftEncoder) - Math.abs(rightEncoder));
        packet.put("Power", power);
        packet.put("Target", target);

        dashboard.sendTelemetryPacket(packet);


        controller.setPID(kp, ki, kd);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, targetArm);
        double ff = Math.cos(Math.toRadians(targetArm / ticks_in_degree)) * kf;

        double powerArm = pid + ff;

        if(gamepad2.a){
            targetArm = 100;
        } else if(gamepad2.b){
            targetArm = 1850;
        } else if(gamepad2.x){
            targetArm = 1000;
        } else if(gamepad2.y){
            targetArm = 0;
        }
        arm.setPower(powerArm);

        telemetry.addData("pos", armPos);
        telemetry.addData("targetArm", targetArm);
        telemetry.update();



    }

    public double pidController(int target, double p, double i, double d) {
        long currentTime = micros();
        double deltaT = ((double)(currentTime +- previousTime)) / 1.0e6;

        int e = rightSlide.getCurrentPosition() - target;
        double eDerivative = (e - ePrevious) / deltaT;
        eIntegral = eIntegral + e * deltaT;

        double u = (p * e) + (d * eDerivative) + (i * eIntegral);

        previousTime = currentTime;
        ePrevious = e;

        return u;
    }

    public long micros() {
        return System.nanoTime() / 1000;
    }

    public void moveMotor(double power) {
        rightSlide.setPower(power);
    }
}