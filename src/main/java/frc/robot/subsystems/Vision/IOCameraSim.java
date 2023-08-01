package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import org.photonvision.SimVisionSystem;

public class IOCameraSim implements VisionIO {
    private final NetworkTable table;
    private final IntegerPublisher publisherInteger;
    private final DoubleArraySubscriber targetPose;
    private final DoubleSubscriber targetArea;
    private final DoubleSubscriber latency;
    private final DoubleSubscriber targetPitch;
    private final DoubleSubscriber targetYaw;
    private final DoubleSubscriber targetSkew;
    private final IntegerSubscriber targetID;
    private final BooleanSubscriber hasTarget
            ;
    SimVisionSystem simVisionSystem;

    public IOCameraSim(String camName) {
        table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(camName);

        publisherInteger = table.getIntegerTopic("pipelineIndex").publish();
        publisherInteger.setDefault(0);

        targetPose = table.getDoubleArrayTopic("targetPose").subscribe(new double[7]);
        targetArea = table.getDoubleTopic("targetArea").subscribe(0);
        latency = table.getDoubleTopic("latencyMillis").subscribe(0);
        targetPitch =  table.getDoubleTopic("targetPitch").subscribe(0);
        targetYaw =  table.getDoubleTopic("targetYaw").subscribe(0);
        targetSkew = table.getDoubleTopic("targetSkew").subscribe(0);
        targetID = table.getIntegerTopic("targetID").subscribe(0);
        hasTarget =  table.getBooleanTopic("hasTarget").subscribe(false);

        simVisionSystem = new SimVisionSystem(
                camName,
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
        var poseTargetOriented = targetPose.get();
        var poseTargetOriented3d = new Pose3d(new Translation3d(poseTargetOriented[0], poseTargetOriented[1], poseTargetOriented[2]), new Rotation3d(new Quaternion(poseTargetOriented[3], poseTargetOriented[4], poseTargetOriented[5], poseTargetOriented[6])));
        var poseFieldOriented3d = getEstimatedPoseFieldOriented(poseTargetOriented3d);
        inputs.poseFieldOriented3d = poseFieldOriented3d;
        inputs.poseFieldOriented = new double[]{poseFieldOriented3d.getX(), poseFieldOriented3d.getY(), poseFieldOriented3d.getZ(), poseFieldOriented3d.getRotation().getQuaternion().getX(), poseFieldOriented3d.getRotation().getQuaternion().getY(), poseFieldOriented3d.getRotation().getQuaternion().getZ(), poseFieldOriented3d.getRotation().getQuaternion().getW()};
        inputs.poseTargetOriented3d = poseTargetOriented3d;
        inputs.poseTargetOriented = poseTargetOriented;
        inputs.area = targetArea.get();
        inputs.latency = latency.get();
        inputs.pitch = targetPitch.get();
        inputs.yaw = targetYaw.get();
        inputs.targetSkew = targetSkew.get();
        inputs.targetID = (int) targetID.get(); //TODO: find real name for target ID in API
        inputs.hasTargets = hasTarget.get();
    }


}
