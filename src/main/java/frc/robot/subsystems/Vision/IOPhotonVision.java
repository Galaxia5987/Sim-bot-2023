package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

public class IOPhotonVision implements VisionIO {

    private final PhotonCamera photonCamera;
    private final PhotonPipelineResult result;
    private final PhotonTrackedTarget target;
    //  private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static IOPhotonVision INSTANCE;


    public IOPhotonVision() {
        PortForwarder.add(5800, "camera", 5800);
        AprilTagFieldLayout fieldLayout;
        photonCamera = new PhotonCamera("Arducam_OV2311_USB_Camera");
        result = photonCamera.getLatestResult();
        target = result.getBestTarget();
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                    PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,  //TODO: check strategies
                    photonCamera,
                    VisionConstants.ROBOT_TO_CAM
            );
//            photonPoseEstimator.setReferencePose(VisionConstants.TARGET_POSITION);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    //    public Pose3d getEstimatedPoseTargetOriented() {
//        var estimatedPose = photonPoseEstimator.update();
//        if (estimatedPose.isPresent()) {
//            System.out.println("ok");
//            return estimatedPose.get().estimatedPose;
//        } else {
//            return null;
//        }
//    }
    public Optional<Pose3d> getEstimatedPoseTargetOriented() {
        PhotonPipelineResult num = photonCamera.getLatestResult();
        System.out.println(num.targets.size());
        num.getTargets().forEach(t -> System.out.println("Ambiguity: " + t.getPoseAmbiguity()));
        return photonPoseEstimator.update(num).map(pose -> pose.estimatedPose);
    }

    //    } public List<PhotonTrackedTarget> getEstimatedPoseTargetOriented() {
//        var estimatedPose = photonPoseEstimator.update();
//        if (estimatedPose.isPresent()) {
//            System.out.println("ok");
//            return estimatedPose.get().targetsUsed;
//        } else {
//            return null;
//        }
//    }
    public void isResult() {
        System.out.println("pose3d x: " + photonPoseEstimator.getRobotToCameraTransform().getX());
        System.out.println("pose3d y: " + photonPoseEstimator.getRobotToCameraTransform().getY());
        System.out.println("pose3d z: " + photonPoseEstimator.getRobotToCameraTransform().getZ());
        System.out.println("pose3d rotation: " + photonPoseEstimator.getRobotToCameraTransform().getRotation());
        System.out.println("field: " + photonPoseEstimator.getFieldTags());
        System.out.println("strategy: " + photonPoseEstimator.getPrimaryStrategy());
        System.out.println("referenced pose: " + photonPoseEstimator.getReferencePose());
    }

// public PhotonPipelineResult isResult(){
//        PhotonPipelineResult photonPipelineResult = photonCamera.getLatestResult();
//        return photonPipelineResult;
//    }


    public static IOPhotonVision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IOPhotonVision();
        }
        return INSTANCE;
    }

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
//        inputs.poseTargetOriented3d = getEstimatedPoseTargetOriented();
        inputs.poseFieldOriented3d = getEstimatedPoseFieldOriented(inputs.poseTargetOriented3d);
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
