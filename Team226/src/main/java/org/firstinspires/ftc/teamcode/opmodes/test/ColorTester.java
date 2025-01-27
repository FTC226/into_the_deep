package org.firstinspires.ftc.teamcode.opmodes.test;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "color")
@Config

public class ColorTester extends LinearOpMode {

    ColorSensor color;


    @Override
    public void runOpMode(){

        color = hardwareMap.get(ColorSensor.class, "Sensor");

        waitForStart();


        while(opModeIsActive()){
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }

    }
}
