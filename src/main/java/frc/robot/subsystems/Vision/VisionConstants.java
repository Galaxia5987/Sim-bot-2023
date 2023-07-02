package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final double CAMERA_HEIGHT = 0;
    public static final double TARGET_HEIGHT_FROM_GROUND = 0;
    public static final double CAMERA_PITCH = 0;
    public static final Transform3d ROBOT_TO_CAM = null;
    public static final Pose3d TARGET_POSITION = new Pose3d();

    public void aprilChooser(int ip) {
        switch (ip) {
            case 1:
                new Pose3d();
                break;
            case 2:
                new Pose3d();
                break;
            case 3:
                new Pose3d();
                break;
            case 4:
                new Pose3d();
                break;
            case 5:
                new Pose3d();
                break;
            case 6:
                new Pose3d();
                break;
        }
    }
}
