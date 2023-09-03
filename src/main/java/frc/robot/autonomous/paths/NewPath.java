package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class NewPath extends SequentialCommandGroup {
    public NewPath(){
        SwerveDrive swerveDrive = SwerveDrive.getInstance();

        addCommands(
                new InstantCommand(swerveDrive::resetPose),
                new InstantCommand(swerveDrive::resetGyro),
                FollowPath.loadTrajectory("Yes")
        );
    }
}
