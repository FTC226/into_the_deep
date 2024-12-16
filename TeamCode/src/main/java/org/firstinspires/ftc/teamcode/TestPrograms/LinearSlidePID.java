package org.firstinspires.ftc.teamcode.TestPrograms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp (name = "Linear Slider With PIDController")
public class LinearSlidePID extends OpMode {

    public DcMotorEx leftSlide, rightSlide;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    public long previousTime = 0;
    public double ePrevious = 0;
    public double eIntegral = 0;

    public static int target = 1000;

    public static double p = 0.03, i = 0.3, d = 0.0002;

    @Override
    public void init() {

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

    }

    @Override
    public void loop() {

        dashboard.sendTelemetryPacket(packet);

        double jS = -gamepad2.left_stick_y;

        int leftEncoder = leftSlide.getCurrentPosition();
        int rightEncoder = rightSlide.getCurrentPosition();

        double power = pidController(leftEncoder, p,i,d);

        leftSlide.setPower(jS);
        moveMotor(power);



//        if (rightEncoder > leftEncoder) {
//            leftSlide.setPower(0);
//            rightSlide.setPower(0);
//        } else {
//        }


        packet.put("p", p);
        packet.put("i", i);
        packet.put("d", d);
        packet.put("Left Motor:", Math.abs(leftEncoder));
        packet.put("Right Motor:", Math.abs(rightEncoder));
        packet.put("Left-Right Motor:", Math.abs(leftEncoder) - Math.abs(rightEncoder));
        packet.put("Power", power);
        packet.put("Target", target);

        dashboard.sendTelemetryPacket(packet);

    }

    public double pidController(int target, double p, double i, double d) {
        long currentTime = micros();
        double deltaT = ((double)(currentTime - previousTime)) / 1.0e6;

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
