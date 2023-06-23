package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

import java.util.Optional;

public interface VisionIO {

    Pose3d getEstimatedPose();

    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputsAutoLogged inputs);

    @AutoLog
    class VisionInputs {
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        int targetID = 0;
        double poseRelativeToTarget = 0;
        Pose3d estimatedPose = new Pose3d();

    }
}
