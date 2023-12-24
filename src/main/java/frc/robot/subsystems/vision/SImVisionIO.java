package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.swerve.SwerveDrive;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SImVisionIO implements VisionIO {
    private final PhotonPoseEstimator estimator;
    Pose3d estimatedPose;
    private VisionSystemSim coprocessorSim;
    private PhotonCameraSim simCamera;
    Transform3d bestCameraToTarget = simCamera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget();


    private SImVisionIO(int camIndex) {
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(90));
        simCamera = new PhotonCameraSim(new PhotonCamera("simCam"), cameraProp);
        coprocessorSim = new VisionSystemSim("simCamSys");
        coprocessorSim.addCamera(simCamera, VisionConstants.ROBOT_TO_CAM[camIndex]);

        try {
            estimator = new PhotonPoseEstimator(
                    AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                    PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
                    simCamera.getCamera(),
                    VisionConstants.ROBOT_TO_CAM[2]
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        simCamera.getCamera().setPipelineIndex(pipeLineIndex);
    }

    @Override
    public Result getLatestResult() {
        return new Result(simCamera.getCamera().getLatestResult().getTimestampSeconds(), new Pose3d(simCamera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(), simCamera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget().getRotation()));
    }

    @Override
    public void updateInputs(VisionInputsAutoLogged inputs) {

        for (int i = 1; i < VisionConstants.TARGET_POSITION_SIM.length; i++) {
            coprocessorSim.addVisionTargets(VisionConstants.SIM_VISION_TARGETS[i]);
            coprocessorSim.update(SwerveDrive.getInstance().getBotPose());
            estimatedPose = estimator.update(simCamera.getCamera().getLatestResult()).get().estimatedPose;
        }

        inputs.hasTargets = simCamera.getCamera().getLatestResult().hasTargets();
        inputs.yaw = simCamera.getCamera().getLatestResult().getBestTarget().getYaw();
        inputs.pitch = simCamera.getCamera().getLatestResult().getBestTarget().getPitch();
        inputs.area = simCamera.getCamera().getLatestResult().getBestTarget().getArea();
        inputs.targetSkew = simCamera.getCamera().getLatestResult().getBestTarget().getSkew();
        inputs.targetID = simCamera.getCamera().getLatestResult().getBestTarget().getFiducialId();
        inputs.cameraToTarget = new double[]{bestCameraToTarget.getX(), bestCameraToTarget.getY(), bestCameraToTarget.getZ(), bestCameraToTarget.getRotation().getX(), bestCameraToTarget.getRotation().getY(), bestCameraToTarget.getRotation().getZ()};
        inputs.poseFieldOriented = new double[]{estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getZ(), estimatedPose.getRotation().getQuaternion().getW(), estimatedPose.getRotation().getQuaternion().getX(), estimatedPose.getRotation().getQuaternion().getY(), estimatedPose.getRotation().getQuaternion().getZ()};
        inputs.fieldOrientedRotationRad = new double[]{estimatedPose.getRotation().getX(), estimatedPose.getRotation().getY(), estimatedPose.getRotation().getZ()};

    }
}

