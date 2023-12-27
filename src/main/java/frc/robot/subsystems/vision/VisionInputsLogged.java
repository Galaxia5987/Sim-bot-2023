package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionInputsLogged implements LoggableInputs {
    long latency = 0;
    boolean hasTargets = false;
    double yaw = 0;
    double pitch = 0;
    double area = 0;
    double targetSkew = 0;
    long targetID = 0;
    double[] cameraToTarget = new double[]{0, 0, 0, 0, 0, 0};
    double[] poseFieldOriented = new double[7];
    double[] fieldOrientedRotationRad = new double[3];

    @Override
    public void toLog(LogTable table) {
        table.put("latency" ,latency);
        table.put("hasTargets" ,hasTargets);
        table.put("yaw" ,yaw);
        table.put("pitch" ,pitch);
        table.put("area" ,area);
        table.put("targetSkew" ,targetSkew);
        table.put("targetID" ,targetID);
        table.put("cameraToTarget" ,cameraToTarget);
        table.put("poseFieldOriented" ,poseFieldOriented);
        table.put("fieldOrientedRotationRad" ,fieldOrientedRotationRad);
    }

    @Override
    public void fromLog(LogTable table) {

    }
}
