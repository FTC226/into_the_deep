package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/** Configuration Notes: CenterStage
 * Port 00: frontLeft
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 */

//@Disabled
@TeleOp (name = "Main Drive")
public class MainDrive extends OpMode {

    FieldCentric Robot = new FieldCentric();

    @Override
    public void init() {
        Robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Gamepad inputs
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX =  gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        boolean resetYaw = gamepad1.x;

        // Field Centric Movement
        Robot.fieldCentric(leftStickX, leftStickY, rightStickX, resetYaw);
        telemetry.addData("Yaw: ", Robot.readIMU());
        telemetry.update();
    }
}
