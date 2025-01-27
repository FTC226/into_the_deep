package org.firstinspires.ftc.teamcode.Subsystems;

public enum LimelightPipeline {
    NONE(-1),
    APRIL_TAG(0),
    SAMPLE_DETECTOR(2);

    public final int pipelineIndex;

    LimelightPipeline(int pipelineIndex) {
        this.pipelineIndex = pipelineIndex;
    }
}