package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.common.swerve.SwerveDrive;
import frc.robot.subsystems.vision.Result;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class RobotState {

    private static RobotState INSTANCE = null;

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Vision vision = Vision.getInstance();

    private final SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
            swerveDrive.getKinematics(),
            new Rotation2d(),
            swerveDrive.getModulePositions(),
            new Pose2d()
    );
    private Pose2d estimatedPose = new Pose2d();

    private RobotState() {
    }

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }
        return INSTANCE;
    }

    public void update() {
        var results = vision.getResults();
        for (Result result : results) {
            if (result != null) {
                estimator.addVisionMeasurement(result.pose.toPose2d(), result.timestamp);
            }
        }

        estimatedPose = estimator.update(
                Rotation2d.fromRadians(swerveDrive.getYaw()),
                swerveDrive.getModulePositions()
        );

        Logger.getInstance().recordOutput("estimatedPose", estimatedPose);
    }

    public void reset() {
        var results = vision.getResults();
        for (Result result : results) {
            if (result != null) {
                estimator.resetPosition(
                        Rotation2d.fromRadians(swerveDrive.getYaw()),
                        swerveDrive.getModulePositions(),
                        result.pose.toPose2d()
                );
                break;
            }
        }
    }
}
