package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.swerve.SwerveDrive;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SImVisionIO implements VisionIO {
    private final PhotonPoseEstimator estimator;
    private VisionSystemSim coprocessorSim;
    private PhotonCameraSim simCamera;

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
        return new Result(simCamera.getCamera().getLatestResult().getTimestampSeconds(), new Pose3d(simCamera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation(),simCamera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget().getRotation()));
    }

    @Override
    public void updateInputs(VisionInputs inputs) {

        for (int i = 1; i < VisionConstants.TARGET_POSITION_SIM.length; i++) {
            coprocessorSim.addVisionTargets(VisionConstants.SIM_VISION_TARGETS[i]);
            coprocessorSim.update(SwerveDrive.getInstance().getBotPose());
        }
        estimator.update();



    }

}
}
