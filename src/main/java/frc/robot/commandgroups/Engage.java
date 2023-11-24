package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillDeltaPitch;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;

public class Engage extends SequentialCommandGroup {
    SwerveDrive swerveDrive = SwerveDrive.getInstance();

    public Engage(double desiredPitch, double xVelocity, double deadband) {
        addCommands(
                new DriveTillPitch(desiredPitch, xVelocity),
                new DriveTillDeltaPitch(deadband, xVelocity)
        );
    }
}
