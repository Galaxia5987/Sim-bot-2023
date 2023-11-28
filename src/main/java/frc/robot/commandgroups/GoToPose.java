package frc.robot.commandgroups;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import swerve.SwerveDrive;

public class GoToPose extends CommandBase {

    private final SwerveDrive swerve = SwerveDrive.getInstance(Robot.isReal());

    private final Pose2d desiredPose;

    private final HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(2, 0, 0),
            new PIDController(2, 0, 0),
            new ProfiledPIDController(3, 0, 0,
                    new TrapezoidProfile.Constraints(Math.PI, Math.PI))
    );

    public GoToPose(Pose2d desiredPose) {
        this.desiredPose = desiredPose;
        addRequirements(swerve);
    }

    /**
     *
     */
    @Override
    public void execute() {
        var estimatedPose = RobotState.getInstance().getEstimatedPose();
        var translation = desiredPose.minus(estimatedPose);
        double velocity = MathUtil.clamp(translation.getTranslation().getNorm(), -4, 4);
        Rotation2d heading = new Rotation2d(translation.getX(), translation.getY());

        swerve.drive(controller.calculate(estimatedPose, desiredPose, velocity, heading), true);
    }
}
