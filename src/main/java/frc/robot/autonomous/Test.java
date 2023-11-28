package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class Test extends CommandBase {
    private final SwerveModule swerveModule;

    public Test(SwerveModule swerveModule) {
        this.swerveModule = swerveModule;
    }

    @Override
    public void execute() {
        swerveModule.checkModule();
    }

    @Override
    public void end(boolean interrupted) {
        swerveModule.setAngleSpeed(0);
        swerveModule.setModuleState(new SwerveModuleState());
    }
}
