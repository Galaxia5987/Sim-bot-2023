package frc.robot.subsystems.Vision;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.Vision.VisionConstants.TARGET_RADIUS;

public class Vision extends SubsystemBase {

    PhotonCamera camera;


    private Vision() {
        camera = new PhotonCamera("PhotonVision");
        PortForwarder.add(5800, "photonvision.local", 5800);
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    }
    public boolean hasTarget() {
        return hasTarget();
    }
    public Optional<Double> getYaw() {
        return Optional.ofNullable(camera.getLatestResult().getBestTarget().getYaw());
    }
    public double getDistance(double cameraHeight, double targetHeight) {
        var results = camera.getLatestResult();
        PhotonUtils.estimateCameraToTarget()
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight,
                    targetHeight,
                    Math.toRadians(VisionConstants.CAMERA_PITCH),
                    Math.toRadians(results.getBestTarget().getPitch())
            );

            return filter.calculate(distance) + TARGET_RADIUS;
        }
        return 0;
    }

    public double getDistance(){
        return 0;
    }
}
