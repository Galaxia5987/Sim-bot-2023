package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.drivetrain.Drive;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

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
                10000,
                1600,
                1200,
                0.0
        );
        for (int i = 0; i < 8; i++) {
            simVisionSystem.addSimVisionTarget(new SimVisionTarget(VisionConstants.TARGET_POSITION_SIM[i+1], VisionConstants.TARGET_WIDTH, VisionConstants.TARGET_LENGTH, i+1));
        }
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
    public Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented, int aprilID) {
//        Transform3d transform3d = new Transform3d(poseTargetOriented.getTranslation(), poseTargetOriented.getRotation());
//        return VisionConstants.TARGET_POSITION_SIM[aprilID].plus(transform3d);
        return VisionIO.super.getEstimatedPoseFieldOriented(poseTargetOriented, aprilID);
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
       // var poseFieldOriented3d = getEstimatedPoseFieldOriented(poseTargetOriented3d, inputs.targetID);
        inputs.poseFieldOriented3d =new Pose3d(new Translation3d(swerveDrive.getCurrentPose().getX(), swerveDrive.getCurrentPose().getY(), 0), new Rotation3d(0, 0, swerveDrive.getCurrentPose().getRotation().getRadians()));
        inputs.poseFieldOriented = new double[]{inputs.poseFieldOriented3d.getX(), inputs.poseFieldOriented3d.getY(), inputs.poseFieldOriented3d.getZ(), poseFieldOriented3d.getRotation().getQuaternion().getX(), poseFieldOriented3d.getRotation().getQuaternion().getY(), poseFieldOriented3d.getRotation().getQuaternion().getZ(), poseFieldOriented3d.getRotation().getQuaternion().getW()};
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
