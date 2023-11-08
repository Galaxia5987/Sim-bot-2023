package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIO implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private Result result;

    public PhotonVisionIO(PhotonCamera camera, Transform3d robotToCamera) {
        this.camera = camera;

        try {
            estimator = new PhotonPoseEstimator(
                    AprilTagFieldLayout.loadFromResource("2023field.fmap"),
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                    camera,
                    robotToCamera
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        var latestResult = camera.getLatestResult();
        inputs.latency = (long) latestResult.getLatencyMillis();
        inputs.area = latestResult.getBestTarget().getArea();
        inputs.pitch = latestResult.getBestTarget().getPitch();
        inputs.yaw = latestResult.getBestTarget().getYaw();
        inputs.targetSkew = latestResult.getBestTarget().getSkew();
        inputs.hasTargets = latestResult.hasTargets();
        inputs.targetID = latestResult.getBestTarget().getFiducialId();

        var estimatedPose = estimator.update(latestResult);
        if (estimatedPose.isPresent()) {
            var pose = estimatedPose.get().estimatedPose;
            inputs.poseFieldOriented = new double[] {
                    pose.getX(),
                    pose.getY(),
                    pose.getZ(),
                    pose.getRotation().getX(),
                    pose.getRotation().getY(),
                    pose.getRotation().getZ()
            };

            var cameraToTarget = latestResult.getBestTarget().getBestCameraToTarget();
            inputs.cameraToTarget = new double[] {
                    cameraToTarget.getX(),
                    cameraToTarget.getY(),
                    cameraToTarget.getZ(),
                    cameraToTarget.getRotation().getX(),
                    cameraToTarget.getRotation().getY(),
                    cameraToTarget.getRotation().getZ()
            };

            result = new Result(latestResult.getTimestampSeconds(), estimatedPose.get().estimatedPose);
        } else {
            result = null;
        }
    }

    @Override
    public Result getLatestResult() {
        return result;
    }
}
