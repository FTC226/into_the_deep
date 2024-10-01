package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3; // Import for Color Sensor V2
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="Color Sensor Test", group="Sensor")
public class ColorSensorTest extends OpMode {

    // Declare Color Sensor
    private ColorSensor colorSensor;
    private boolean useLED = false;

    @Override
    public void init() {
        // Initialize the color sensor from the hardware map
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        // Read color values from the sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Output the RGB values to telemetry
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // Using the LED
        changeLED();

        // Color Detection Logic

        telemetry.update();
    }

    public void changeLED() {
        if (gamepad1.b) {
            useLED = !useLED;
            colorSensor.enableLed(useLED);
        }

        telemetry.addData("Using LED: ", useLED);
    }
}
