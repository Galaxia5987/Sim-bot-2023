package frc.robot.subsystems.drivetrain.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class DriveTest2 extends CommandBase {
    private  SwerveDrive swerveDrive;
    private final SwerveModuleState swerveModuleStates4 = new SwerveModuleState(-1, Rotation2d.fromDegrees(-45) );

    public DriveTest2(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.getSwerveModule(2).setModuleState(swerveModuleStates4);
    }
}
