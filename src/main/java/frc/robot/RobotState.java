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
        //drivePoseEstimator.update(swerveDrive.getCurrentAngle(), driveIOInputs.currentModuleStates); TODO: add to swerve

//        var measurement = vision.getEstimatedGlobal1_3d().toPose2d();
//        if (measurement != null) {
//            drivePoseEstimator.addVisionMeasurement(measurement, Timer.getFPGATimestamp());
//        }
    }
}
