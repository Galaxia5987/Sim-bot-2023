package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputs inputs);

    default Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented) {
        Transform3d transform3d = new Transform3d(poseTargetOriented.getTranslation(), poseTargetOriented.getRotation());
        return VisionConstants.TARGET_POSITION.plus(transform3d);
    }

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
        Pose3d poseTargetOriented3d = new Pose3d();
        Pose3d poseFieldOriented3d = new Pose3d();

    }
}
