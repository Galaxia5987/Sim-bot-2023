package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private final VisionIO.VisionInputs inputs;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    PhotonPipelineResult result;
    boolean hasTargets;
    double yaw;
    double pitch;
    double area;
    double skew;
    PhotonTrackedTarget target;
    int targetID;
    double poseAmbiguity;
    Transform3d alternateCameraToTarget;
    private Vision() {
        camera = new PhotonCamera("PhotonVision");
        PortForwarder.add(5800, "photonvision.local", 5800);
        AprilTagFieldLayout fieldLayout;
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
        result = camera.getLatestResult();
        target = result.getBestTarget();
        hasTargets = result.hasTargets();
        yaw = target.getYaw();
        pitch = target.getPitch();
        area = target.getArea();
        skew = target.getSkew();
        targetID = target.getFiducialId();
        poseAmbiguity = target.getPoseAmbiguity();
        alternateCameraToTarget = target.getAlternateCameraToTarget();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonPoseEstimator.update();
    }

    public void updateInputs(){

    }
}
