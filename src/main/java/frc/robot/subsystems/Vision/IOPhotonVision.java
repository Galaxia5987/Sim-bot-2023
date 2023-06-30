package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

public class IOPhotonVision implements VisionIO {

    private final PhotonCamera photonCamera;
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    AprilTagFieldLayout fieldLayout;
    PhotonPoseEstimator photonPoseEstimator;

    public IOPhotonVision(PhotonCamera camera) {
        this.photonCamera = new PhotonCamera("PhotonVision");
        result = camera.getLatestResult();
        target = result.getBestTarget();
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,  //TODO: check strategies
                    camera, VisionConstants.ROBOT_TO_CAM);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public Pose3d getEstimatedPoseTargetOriented() {
        var estimatedPose = photonPoseEstimator.update();
        return estimatedPose.map(estimatedRobotPose -> estimatedRobotPose.estimatedPose).orElse(null);
    }

    public Pose3d getEstimatedPoseFieldOriented() {
        Transform3d transform3d = new Transform3d(getEstimatedPoseTargetOriented().getTranslation(), getEstimatedPoseTargetOriented().getRotation());
        var estimatedPose = VisionConstants.TARGET_POSITION.plus(transform3d);// Todo: add position variation for all aprilTags ;)
        return estimatedPose;
    }

// From Barel 😘:
// code code code coding the code with some code to fix the code becasue i want code, code code code... all day long - just code!

    public void setPipeLine(int pipeLineIndex) {
        photonCamera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputsAutoLogged inputs) {
        inputs.targetID = target.getFiducialId();
        inputs.hasTargets = result.hasTargets();
        inputs.yaw = target.getYaw();
        inputs.pitch = target.getPitch();
        inputs.targetSkew = target.getSkew();
        inputs.area = target.getArea();
        inputs.latency = 0;
        Pose3d targetOriented = getEstimatedPoseTargetOriented();
        Pose3d robotOriented = getEstimatedPoseFieldOriented();
        inputs.poseTargetOriented = new double[]{targetOriented.getX(), targetOriented.getY(), targetOriented.getZ(), targetOriented.getRotation().getQuaternion().getX(), targetOriented.getRotation().getQuaternion().getY(), targetOriented.getRotation().getQuaternion().getZ(), targetOriented.getRotation().getQuaternion().getW()};
        inputs.poseFieldOriented = new double[]{robotOriented.getX(), robotOriented.getY(), robotOriented.getZ(), robotOriented.getRotation().getQuaternion().getX(), robotOriented.getRotation().getQuaternion().getY(), robotOriented.getRotation().getQuaternion().getZ(), robotOriented.getRotation().getQuaternion().getW()};

    }
}