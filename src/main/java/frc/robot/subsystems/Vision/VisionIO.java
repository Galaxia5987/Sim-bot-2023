package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

import java.util.Optional;

public interface VisionIO {

    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputsAutoLogged inputs);

    @AutoLog
    class VisionInputs {
        double latency = 0;
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        int targetID = 0;
        double[] poseTargetOriented = new double[7];
        double[] poseFieldOriented = new double[7];
        double[] estimatedPose = new double[7];

    }
}
