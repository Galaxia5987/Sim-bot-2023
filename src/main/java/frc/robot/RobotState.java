package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.drivetrain.Drive;

public class RobotState {
    private static RobotState INSTANCE = null;
    private Drive swerveDrive;
    private Vision vision;
    private Drive.DriveIOInputs driveIOInputs;
    private SwerveDrivePoseEstimator drivePoseEstimator;


    public static RobotState getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public void update() {
        var measurements = vision.getEstimatedPoses();
        for (int i = 0; i <= measurements.length; i++) {
            drivePoseEstimator.update(swerveDrive.getCurrentAngle(), swerveDrive.getCurrentModulePositions());

            if (measurements != null) {
                drivePoseEstimator.addVisionMeasurement(measurements[i].toPose2d(), Timer.getFPGATimestamp());
            }

            if (i > measurements.length) {
            }
        }

    }

}
