package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Optional;

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
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,  //TODO: check strategies
                    camera,
                    VisionConstants.ROBOT_TO_CAM
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }


    @Override
    public Pose3d getEstimatedPose() {
        var estimatedPose = photonPoseEstimator.update();
        return estimatedPose.map(estimatedRobotPose -> estimatedRobotPose.estimatedPose).orElse(null);
    }

    @Override
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
        inputs.estimatedPose = getEstimatedPose();
    }
}