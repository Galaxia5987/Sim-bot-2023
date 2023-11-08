package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public class Result {

    private final long latency;
    private final Pose3d pose;

    public Result(long latency, Pose3d pose) {
        this.latency = latency;
        this.pose = pose;
    }
}
