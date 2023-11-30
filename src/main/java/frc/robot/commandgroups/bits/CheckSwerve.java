package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import swerve.SwerveDrive;

public class CheckSwerve extends SequentialCommandGroup {
    public CheckSwerve() {
        SwerveDrive swerveDrive = SwerveDrive.getInstance(Robot.isReal());
        addCommands(
                new RunCommand(swerveDrive::checkSwerve, swerveDrive).withTimeout(5),
                new ZeroPositionSwerve().withTimeout(2)
        );
    }
}



