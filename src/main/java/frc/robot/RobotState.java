package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.SwerveConstants;

public class RobotState {
    private static RobotState INSTANCE;
    private Drive swerveDrive = Drive.getInstance();
    private Vision vision = Vision.getINSTANCE();
    private SwerveDrivePoseEstimator drivePoseEstimator = new SwerveDrivePoseEstimator( swerveDrive.getKinematics(), swerveDrive.getCurrentAngle(), swerveDrive.getCurrentModulePositions(), new Pose2d());


    public static RobotState getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public void update() {
        var measurements = vision.getEstimatedPoses();
        for (int i = 0; i < measurements.length; i++) {
            if (measurements[i] != null) {
                drivePoseEstimator.addVisionMeasurement(measurements[i].toPose2d(), Timer.getFPGATimestamp());
            } else {
                drivePoseEstimator.addVisionMeasurement(new Pose2d(), Timer.getFPGATimestamp());
            }
            drivePoseEstimator.update(swerveDrive.getCurrentAngle(), swerveDrive.getCurrentModulePositions());


        }
    }

}
