package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.FieldCentric;
import org.firstinspires.ftc.teamcode.FieldCentricModified;

public class straightFCTest extends OpMode {


    FieldCentricModified Robot = new FieldCentricModified();

    @Override
    public void init() {
        Robot.init(hardwareMap);
        Robot.fieldCentric(0,0,0, true);

    }

    @Override
    public void loop() {
        double frontPower = -gamepad1.left_stick_y;
        double sidePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;
        boolean resetIMU = gamepad1.x;


        Robot.fieldCentric(frontPower,sidePower,turnPower, resetIMU);



        telemetry.addData("Direction", "<" + frontPower + "," + sidePower + ">");
        telemetry.addData("Magnitude", Math.sqrt(Math.pow(frontPower,2) + Math.pow(sidePower,2)));
        telemetry.addData("Heading", Robot.readIMU());
        telemetry.update();
    }
}
