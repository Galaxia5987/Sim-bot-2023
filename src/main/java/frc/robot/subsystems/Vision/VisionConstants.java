package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

public class VisionConstants {
    public static final Transform3d[] ROBOT_TO_CAM = new Transform3d[]{new Transform3d( new Translation3d(0,0,0.45), new Rotation3d(0,0, 270)), new Transform3d( new Translation3d(0,0,0.45), new Rotation3d(0,180, 0))};
public static final Transform3d PHOTON_OFFSET = new Transform3d(new Translation3d((16.54)/2, (8.02)/2, 0 ), new Rotation3d());


}
