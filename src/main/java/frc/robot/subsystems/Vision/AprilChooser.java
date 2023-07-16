package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilChooser {
    int aprilId;

    public AprilChooser(int aprilId) {
        this.aprilId = aprilId;
    }

    public void getSpecifiedTarget() {
        switch (this.aprilId) {
            case 0:
                VisionConstants.TARGET_POSITION = new Pose3d();
            case 1:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 2:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 3:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 4:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 5:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 6:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 7:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 8:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
            case 9:
                VisionConstants.TARGET_POSITION = new Pose3d();
                break;
                //TODO: check real target position...not finished
        }
    }
}
