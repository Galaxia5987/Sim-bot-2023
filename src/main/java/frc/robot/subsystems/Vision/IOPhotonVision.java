package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;

public class IOPhotonVision implements VisionIO {

    private final PhotonCamera photonCamera;
    private final PhotonPipelineResult result;
    private final PhotonTrackedTarget target;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator photonPoseEstimator;

    public IOPhotonVision(PhotonCamera camera, int camIndex) {
        this.photonCamera = camera;
        result = photonCamera.getLatestResult();
        target = result.getBestTarget();
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,  //TODO: check strategies
                    photonCamera, VisionConstants.ROBOT_TO_CAM[camIndex]);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public Pose3d getEstimatedPoseTargetOriented() {
        var estimatedPose = photonPoseEstimator.update();
        return estimatedPose.map(estimatedRobotPose -> estimatedRobotPose.estimatedPose).orElse(null);
    }

    @Override
    public Pose3d getEstimatedPoseFieldOriented(Pose3d poseTargetOriented, int aprilId) {
        Transform3d transform3d = new Transform3d(poseTargetOriented.getTranslation(), poseTargetOriented.getRotation());
        return this.aprilChooser(aprilId).plus(transform3d);
    }

    @Override
    public Pose3d aprilChooser(int aprilID) {
        return VisionIO.super.aprilChooser(aprilID).plus(VisionConstants.PHOTON_OFFSET);
    }

    // From Barel 😘:
// code code code coding the code with some code to fix the code becasue i want code, code code code... all day long - just code!

    public void setPipeLine(int pipeLineIndex) {
        photonCamera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.targetID = target.getFiducialId();
        inputs.hasTargets = result.hasTargets();
        inputs.yaw = target.getYaw();
        inputs.pitch = target.getPitch();
        inputs.targetSkew = target.getSkew();
        inputs.area = target.getArea();
        inputs.latency = 0;
        inputs.poseTargetOriented3d = getEstimatedPoseTargetOriented();
        inputs.poseFieldOriented3d = getEstimatedPoseFieldOriented(inputs.poseTargetOriented3d, inputs.targetID);
        inputs.poseTargetOriented = new double[]{
                inputs.poseTargetOriented3d.getX(),
                inputs.poseTargetOriented3d.getY(),
                inputs.poseTargetOriented3d.getZ(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getX(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getY(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getZ(),
                inputs.poseTargetOriented3d.getRotation().getQuaternion().getW()};
        inputs.poseFieldOriented = new double[]{
                inputs.poseFieldOriented3d.getX(),
                inputs.poseFieldOriented3d.getY(),
                inputs.poseFieldOriented3d.getZ(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getX(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getY(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getZ(),
                inputs.poseFieldOriented3d.getRotation().getQuaternion().getW()};

    }
}