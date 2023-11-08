package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {


    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputs inputs);

    default Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented, int aprilId) {
        Transform3d transform3d = new Transform3d(poseTargetOriented.getTranslation(), poseTargetOriented.getRotation());
        if(Robot.isReal()){
            return VisionConstants.TARGET_POSITION_REAL[aprilId].plus(transform3d);

        }
        else {
            return VisionConstants.TARGET_POSITION_SIM[aprilId].plus(transform3d);
        }
    }


    }

    @AutoLog
    class VisionInputs {
        double latency = 0;
        boolean hasTargets = false;
        double yaw = 0;
        double pitch = 0;
        double area = 0;
        double targetSkew = 0;
        long targetID = 0;
        double[] poseTargetOriented = new double[7];
        double[] poseFieldOriented = new double[7];
        double[] targetFieldOriented = new double[7];
    }
