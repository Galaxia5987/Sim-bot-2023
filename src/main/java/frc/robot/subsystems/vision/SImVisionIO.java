package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import swerve.SwerveDrive;

public class SImVisionIO implements VisionIO{
    private SimPhotonCamera simCamera;
    private final PhotonPoseEstimator estimator;
    private SimVisionSystem coprocessorSim;
    private SImVisionIO(){
        simCamera = new SimPhotonCamera(simCamera.getCameraTable().getInstance() , "simCam");
        coprocessorSim = new SimVisionSystem("simCam", 0,0, new Transform2d(VisionConstants.ROBOT_TO_CAM[2].getTranslation().toTranslation2d(), VisionConstants.ROBOT_TO_CAM[2].getTranslation().toTranslation2d().getAngle()), VisionConstants.ROBOT_TO_CAM[2].getZ(), 10000, 54, 41, 0);

        try {
            estimator = new PhotonPoseEstimator(
                    AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(),
                    PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
                    simCamera,
                    VisionConstants.ROBOT_TO_CAM[2]
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }








    @Override
    public void setPipeLine(int pipeLineIndex) {

    }

    @Override
    public Result getLatestResult() {
        return null;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        for (int i = 1; i < VisionConstants.TARGET_POSITION_SIM.length; i++) {
            coprocessorSim.addSimVisionTarget(VisionConstants.SIM_VISION_TARGETS[i]);
            coprocessorSim.processFrame(SwerveDrive.getInstance(false).getBotPose());
        }
       estimator.update(SwerveDrive.getInstance(false).getRotation2d(), leftDist, rightDist);

        var res = cam.getLatestResult();
        if (res.hasTargets()) {
            var imageCaptureTime = res.getTimestampSeconds();
            var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            m_poseEstimator.addVisionMeasurement(
                    camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);
        }

    }
}
