package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.drivetrain.Drive;
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
    private final BooleanSubscriber hasTarget;
    SimVisionSystem simVisionSystem;
    private final Drive swerveDrive = Drive.getInstance();

    public IOCameraSim(int camIndex, String camName) {
        table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(camName);

        publisherInteger = table.getIntegerTopic("pipelineIndex").publish();
        publisherInteger.setDefault(0);

        simVisionSystem = new SimVisionSystem(
                camName,
                90.0,
                VisionConstants.ROBOT_TO_CAM[camIndex],
                7,
                1600,
                1200,
                0.0
        );
        targetPose = table.getDoubleArrayTopic("targetPose").subscribe(new double[7]);
        targetArea = table.getDoubleTopic("targetArea").subscribe(0);
        latency = table.getDoubleTopic("latencyMillis").subscribe(0);
        targetPitch = table.getDoubleTopic("targetPitch").subscribe(0);
        targetYaw = table.getDoubleTopic("targetYaw").subscribe(0);
        targetSkew = table.getDoubleTopic("targetSkew").subscribe(0);
        targetID = table.getIntegerTopic("targetID").subscribe(0);
        hasTarget = table.getBooleanTopic("hasTarget").subscribe(false);
    }

    @Override
    public Pose3d AprilChooser(int aprilID) {
        return VisionIO.super.AprilChooser(aprilID).plus(new Transform3d(new Translation3d((16.54)/2, (8.02)/2, 0 ), new Rotation3d()));
    }

    @Override
    public Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented, int aprilID) {
        Transform3d transform3d = new Transform3d(poseTargetOriented.getTranslation(), poseTargetOriented.getRotation());
        return this.AprilChooser(aprilID).plus(transform3d);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        publisherInteger.set(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        simVisionSystem.processFrame(swerveDrive.getCurrentPose());
        var poseTargetOriented = targetPose.get();
        var poseTargetOriented3d = new Pose3d(new Translation3d(poseTargetOriented[0], poseTargetOriented[1], 0), new Rotation3d(0,0,poseTargetOriented[2]));
        //var poseTargetOriented3d = new Pose3d(new Translation3d(poseTargetOriented[0], poseTargetOriented[1], poseTargetOriented[2]), new Rotation3d(new Quaternion(poseTargetOriented[3], poseTargetOriented[4], poseTargetOriented[5], poseTargetOriented[6])));
        var poseFieldOriented3d = getEstimatedPoseFieldOriented(poseTargetOriented3d, inputs.targetID);
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
