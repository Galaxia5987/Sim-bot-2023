package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import org.photonvision.SimVisionSystem;

public class IOCameraSim implements VisionIO {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("simCamera");
    private IntegerPublisher publisherInteger = table.getIntegerTopic("pipelineIndex").publish();
    SimVisionSystem simVisionSystem;

    public IOCameraSim() {
        publisherInteger.setDefault(0);
        simVisionSystem = new SimVisionSystem(
                "simCamera",
                90.0,
                VisionConstants.ROBOT_TO_CAM,
                7,
                1600,
                1200,
                0.0
        );
    }

    @Override
    public Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented) {
        return VisionIO.super.getEstimatedPoseFieldOriented(poseTargetOriented);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
         publisherInteger.set(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        var poseTargetOriented = table.getDoubleArrayTopic("targetPose").getEntry(new double[7]).get();
        var poseTargetOriented3d = new Pose3d(new Translation3d(poseTargetOriented[0], poseTargetOriented[1], poseTargetOriented[2]), new Rotation3d(new Quaternion(poseTargetOriented[3], poseTargetOriented[4], poseTargetOriented[5], poseTargetOriented[6])));
        var poseFieldOriented3d = getEstimatedPoseFieldOriented(poseTargetOriented3d);
        inputs.poseFieldOriented3d = poseFieldOriented3d;
        inputs.poseFieldOriented = new double[]{poseFieldOriented3d.getX(), poseFieldOriented3d.getY(), poseFieldOriented3d.getZ(), poseFieldOriented3d.getRotation().getQuaternion().getX(), poseFieldOriented3d.getRotation().getQuaternion().getY(), poseFieldOriented3d.getRotation().getQuaternion().getZ(), poseFieldOriented3d.getRotation().getQuaternion().getW()};
        inputs.poseTargetOriented3d = poseTargetOriented3d;
        inputs.poseTargetOriented = poseTargetOriented;
        inputs.area = table.getDoubleTopic("targetArea").getEntry(0).get();
        inputs.latency = table.getDoubleTopic("latencyMillis").getEntry(0).get();
        inputs.pitch = table.getDoubleTopic("targetPitch").getEntry(0).get();
        inputs.yaw = table.getDoubleTopic("targetYaw").getEntry(0).get();
        inputs.targetSkew = table.getDoubleTopic("targetSkew").getEntry(0).get();
        inputs.targetID = (int) table.getIntegerTopic("targetID").getEntry(0).get(); //TODO: find real name for target ID in API
        inputs.hasTargets = table.getBooleanTopic("targetID").getEntry(false).get();
    }


}
