package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonPoseEstimator photonPoseEstimator1;
    private final PhotonPoseEstimator photonPoseEstimator2;
    PhotonPipelineResult result1;
    PhotonPipelineResult result2;
    PhotonTrackedTarget target1;
    PhotonTrackedTarget target2;
    double poseAmbiguity1;
    double poseAmbiguity2;
    Transform3d alternateCameraToTarget1;
    Transform3d alternateCameraToTarget2;
    private VisionInputsAutoLogged inputs;

    private Vision() {
        camera1 = new PhotonCamera("PhotonVision_1");
        camera2 = new PhotonCamera("PhotonVision_2");
        PortForwarder.add(5800, "photonvision.local", 5800);
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator1 = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,  //TODO: check strategies
                    camera1,
                    VisionConstants.ROBOT_TO_CAM
            );
            photonPoseEstimator2 = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,  //TODO: check strategies
                    camera2,
                    VisionConstants.ROBOT_TO_CAM
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        result1 = camera1.getLatestResult();
        result2 = camera2.getLatestResult();
        target1 = result1.getBestTarget();
        target2 = result2.getBestTarget();
        poseAmbiguity1 = target1.getPoseAmbiguity();
        poseAmbiguity2 = target2.getPoseAmbiguity();
        alternateCameraToTarget1 = target1.getAlternateCameraToTarget();
        alternateCameraToTarget2 = target2.getAlternateCameraToTarget();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose1() {
        return photonPoseEstimator1.update();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose2() {
        return photonPoseEstimator1.update();
    }

    @Override
    public void periodic() {
        inputs.hasTargets1 = result1.hasTargets();
        inputs.hasTargets2 = result2.hasTargets();
        inputs.hasTargets1 = result1.hasTargets();
        inputs.hasTargets2 = result2.hasTargets();
        inputs.yaw1 = target1.getYaw();
        inputs.yaw2 = target2.getYaw();
        inputs.pitch1 = target1.getPitch();
        inputs.pitch2 = target2.getPitch();
        inputs.area1 = target1.getArea();
        inputs.area2 = target2.getArea();
        inputs.skew1 = target1.getSkew();
        inputs.skew2 = target2.getSkew();
        inputs.targetID_1 = target1.getFiducialId();
        inputs.targetID_2 = target2.getFiducialId();
        inputs.poseAmbiguity1 = target1.getPoseAmbiguity();
        inputs.poseAmbiguity2 = target2.getPoseAmbiguity();
    }

    @AutoLog
    public static class VisionInputs {
        boolean hasTargets1 = false;
        boolean hasTargets2 = false;
        double yaw1 = 0;
        double yaw2 = 0;
        double pitch1 = 0;
        double pitch2 = 0;
        double area1 = 0;
        double area2 = 0;
        double skew1 = 0;
        double skew2 = 0;
        int targetID_1 = 0;
        int targetID_2 = 0;
        double poseAmbiguity1 = 0;
        double poseAmbiguity2 = 0;
    }
}
