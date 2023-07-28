package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

public class IOPhotonVision implements VisionIO {

    private final PhotonCamera photonCamera;
    private final PhotonPipelineResult result;
    private final PhotonTrackedTarget target;
  //  private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static IOPhotonVision INSTANCE;


    public IOPhotonVision() {
        PortForwarder.add(5800, "camera", 5800);
        AprilTagFieldLayout fieldLayout;
        photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        result = photonCamera.getLatestResult();
        target = result.getBestTarget();
//        fieldLayout = photonCamera.
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,  //TODO: check strategies
                    photonCamera,
                    VisionConstants.ROBOT_TO_CAM
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
//
    public Pose3d getEstimatedPoseTargetOriented() {
        var estimatedPose = photonPoseEstimator.update();
        if (estimatedPose.isPresent()) {
            System.out.println("ok");
            return estimatedPose.get().estimatedPose;
        } else {
            return null;
        }
    }
    public boolean isResult(){
        return result.hasTargets();
    }


    public static IOPhotonVision getInstance(){
        if (INSTANCE == null){
            INSTANCE = new IOPhotonVision();
        }
        return INSTANCE;
    }

    public void setPipeLine(int pipeLineIndex) {
        photonCamera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.targetID = target.getFiducialId();
        inputs.hasTargets = result.hasTargets();
        inputs.yaw = target.getYaw();
        inputs.pitch = target.getPitch();
        inputs.targetSkew = target.getSkew();
        inputs.area = target.getArea();
        inputs.latency = 0;
        inputs.poseTargetOriented3d = getEstimatedPoseTargetOriented();
        inputs.poseFieldOriented3d = getEstimatedPoseFieldOriented(inputs.poseTargetOriented3d);
        inputs.poseTargetOriented = new double[]{
                inputs.poseTargetOriented3d.getX(),
                inputs.poseTargetOriented3d.getY(),
                inputs.poseTargetOriented3d.getZ(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getX(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getY(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getZ(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getW()};
        inputs.poseFieldOriented = new double[]{
                inputs.poseFieldOriented3d.getX(),
                inputs.poseFieldOriented3d.getY(),
                inputs.poseFieldOriented3d.getZ(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getX(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getY(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getZ(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getW()};

    }
}