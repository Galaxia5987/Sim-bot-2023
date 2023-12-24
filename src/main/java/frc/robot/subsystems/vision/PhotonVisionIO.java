package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIO implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private Result result;

    public PhotonVisionIO(PhotonCamera camera, Transform3d robotToCamera) {
        this.camera = camera;
        camera.setPipelineIndex(0);

        try {
            estimator = new PhotonPoseEstimator(
                    AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
//                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                    PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
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
    public void updateInputs(VisionInputsAutoLogged inputs) {
        var latestResult = camera.getLatestResult();
        for (int i = 0; i < latestResult.targets.size(); i++) {
        }

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
                        pose.getRotation().getQuaternion().getW(),
                        pose.getRotation().getQuaternion().getX(),  //roll
                        pose.getRotation().getQuaternion().getY(), //pitch
                        pose.getRotation().getQuaternion().getZ() //yaw
                };
                inputs.fieldOrientedRotationRad = new double[]{
                        pose.getRotation().getX(),  //roll
                        pose.getRotation().getY(), //pitch
                        pose.getRotation().getZ() //yaw
                };

                result = new Result(latestResult.getTimestampSeconds(), estimatedPose.get().estimatedPose);
            } else {
                result = null;
            }
        } else {
            result = null;
        }
    }

    @Override
    public Result getLatestResult() {
        return result;
    }
}
