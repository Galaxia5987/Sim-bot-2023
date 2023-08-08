package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

public class VisionConstants {
    public static final double CAMERA_HEIGHT = 0;
    public static final double TARGET_HEIGHT_FROM_GROUND = 0;
    public static final double CAMERA_PITCH = 0;
    public static final Transform3d[] ROBOT_TO_CAM = new Transform3d[]{new Transform3d( new Translation3d(0,0,0.8), new Rotation3d(0,0, 45)), new Transform3d( new Translation3d(0,0,0.8), new Rotation3d(0,0, -45))};
    public static Pose3d TARGET_POSITION = new Pose3d();



}
