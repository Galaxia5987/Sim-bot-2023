package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public static final double CAMERA_HEIGHT = 0.92;
    public static final double TARGET_HEIGHT_FROM_GROUND = 1.2;
    public static final double CAMERA_PITCH = 0;
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Pose3d(1,1,1, new Rotation3d(4,0,0)),new Pose3d(0,0,0, new Rotation3d(3,4,6)));
    public static Pose3d TARGET_POSITION = new Pose3d(1, 1.2, 1, new Rotation3d(0,0,0));



}

