package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSimIO implements VisionIO {
    private SimVisionSystem simVisionSystem;
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget trackedTarget;
    private PhotonTrackedTarget Targets;
    SimPhotonCamera

    public VisionSimIO(Transform3d robotToCam) {
        simVisionSystem = new SimVisionSystem("simCam", 95, robotToCam, 1000, 1600, 1200, 0);
        camera = new PhotonCamera(NetworkTableInstance.getDefault(), "simCam");
        try {
            simVisionSystem.addVisionTargets(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
        } catch (Exception e) {
            e.printStackTrace();
        }
        result = camera.getLatestResult();
        trackedTarget = result.getBestTarget();
    }


    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public Result getLatestResult() {
        return new Result(result.getTimestampSeconds(), new Pose3d(new Translation3d(trackedTarget.getAlternateCameraToTarget().getX(), trackedTarget.getAlternateCameraToTarget().getY(), trackedTarget.getAlternateCameraToTarget().getZ()), new Rotation3d(trackedTarget.getAlternateCameraToTarget().getRotation().getX(), trackedTarget.getAlternateCameraToTarget().getRotation().getY(), trackedTarget.getAlternateCameraToTarget().getRotation().getZ())));
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.yaw = trackedTarget.getYaw();
        inputs.targetSkew = trackedTarget.getSkew();
        inputs.pitch = trackedTarget.getPitch();
        inputs.targetID = trackedTarget.getFiducialId();
        inputs.hasTargets = result.hasTargets();
        inputs.area = trackedTarget.getArea();
        inputs.latency = (long) result.getLatencyMillis();
        inputs.cameraToTarget = new double[]{trackedTarget.getAlternateCameraToTarget().getX(), trackedTarget.getAlternateCameraToTarget().getY(), trackedTarget.getAlternateCameraToTarget().getZ(), trackedTarget.getAlternateCameraToTarget().getRotation().getQuaternion().getW(), trackedTarget.getAlternateCameraToTarget().getRotation().getQuaternion().getX(), trackedTarget.getAlternateCameraToTarget().getRotation().getQuaternion().getY(), trackedTarget.getAlternateCameraToTarget().getRotation().getQuaternion().getZ()};
        inputs.poseFieldOriented = new double[]{};

    }

}
