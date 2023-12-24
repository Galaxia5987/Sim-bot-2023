package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {


    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputsAutoLogged inputs);

    Result getLatestResult();

    @AutoLog
    class VisionInputs {
        long latency = 0;
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        long targetID = 0;
        double[] cameraToTarget = new double[]{0,0,0,0,0,0};
        double[] poseFieldOriented = new double[7];
        double[] fieldOrientedRotationRad = new double[3];
    }
}
