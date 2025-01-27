package org.firstinspires.ftc.teamcode.subss;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Point;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

public class LimeLightTest {
     Limelight3A lime;
    LLResult result = lime.getLatestResult();
    LLResultTypes.DetectorResult results = result.getDetectorResults().get(0);
    public double targetArea = 0;
    OpMode opMode;

    public LimeLightTest(OpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        lime = hardwareMap.get(Limelight3A.class, "lime");
        telemetry.setMsTransmissionInterval(11);
        lime.pipelineSwitch(0);
        lime.start();
    }


    public void updateTelemetry() {
        if (result.isValid()) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
            telemetry.addData("Sample:", result.getClassifierResults());

            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
        }
    }

    public boolean isCorner(){
        List<List<Double>> a = results.getTargetCorners();
        Double firstCornerX = a.get(0).get(0);
        Double firstCornerY = a.get(1).get(0);
        Double rightCornerX = a.get(0).get(1);
        Double bottomCornerY = a.get(1).get(4);

        if(rightCornerX - firstCornerX > firstCornerY - bottomCornerY){
            return true;
        }
        return false;
    }

    public boolean isAligned(){
        if(result.getTx() > -3 && result.getTx() < 3){
            return true;
        }
        return false;
    }

    public boolean toLeft(){
        if(result.getTx() < -3){
            return true;
        }
        return false;
    }

    public boolean toDown(){
        if(result.getTx() < -3){
            return true;
        }
        return false;
    }

    public boolean isClose(){
        if(targetArea > 65){ //target area to be adjusted
            return true;
        }
        return false;
    }
}
