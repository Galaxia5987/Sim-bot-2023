package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSimIO implements VisionIO {
    private SimVisionSystem simVisionSystem;
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    private PhotonTrackedTarget trackedTarget = new PhotonTrackedTarget();
    private PhotonPoseEstimator estimator;
    private Result result;

    public VisionSimIO(Transform3d robotToCam) {
        simVisionSystem = new SimVisionSystem("simCam", 95, robotToCam, 1000, 1600, 1200, 0);
        camera = new PhotonCamera(NetworkTableInstance.getDefault(), "simCam");
        try {
            simVisionSystem.addVisionTargets(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
            estimator = new PhotonPoseEstimator(
                    AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
//                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                    PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
                    camera,
                    robotToCam
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public Result getLatestResult() {
        return result;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        latestResult = camera.getLatestResult();
        if (latestResult != null) {
            inputs.latency = (long) latestResult.getLatencyMillis();
            inputs.hasTargets = latestResult.hasTargets();

            if (latestResult.getBestTarget() != null) {
                inputs.area = latestResult.getBestTarget().getArea();
                inputs.pitch = latestResult.getBestTarget().getPitch();
                inputs.yaw = latestResult.getBestTarget().getYaw();
                inputs.targetSkew = latestResult.getBestTarget().getSkew();
                inputs.targetID = latestResult.getBestTarget().getFiducialId();

                var cameraToTarget = latestResult.getBestTarget().getBestCameraToTarget();
                inputs.cameraToTarget = new double[]{
                        cameraToTarget.getX(),
                        cameraToTarget.getY(),
                        cameraToTarget.getZ(),
                        cameraToTarget.getRotation().getX(),
                        cameraToTarget.getRotation().getY(),
                        cameraToTarget.getRotation().getZ()
                };
            }

            var estimatedPose = estimator.update(latestResult);
            if (estimatedPose.isPresent()) {
                var pose = estimatedPose.get().estimatedPose;
                inputs.poseFieldOriented = new double[]{
                        pose.getX(),
                        pose.getY(),
                        pose.getZ(),
                        pose.getRotation().getX(),
                        pose.getRotation().getY(),
                        pose.getRotation().getZ()
                };

                result = new Result(latestResult.getTimestampSeconds(), estimatedPose.get().estimatedPose);
            } else {
                latestResult = null;
            }
        } else {
            latestResult = null;
        }
    }

}
