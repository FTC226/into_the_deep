package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

public class LimeLightTest extends LinearOpMode {
    private Limelight3A lime;
    public void runOpMode() throws InterruptedException
    {
        lime = hardwareMap.get(Limelight3A.class, "lime");

        telemetry.setMsTransmissionInterval(11);

        lime.pipelineSwitch(0);

        lime.start();

        LLResult result = lime.getLatestResult();
        while(opModeIsActive()){
            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        }

        telemetry.update();
    }
}
