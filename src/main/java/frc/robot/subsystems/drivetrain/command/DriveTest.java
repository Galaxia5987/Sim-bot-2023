package frc.robot.subsystems.drivetrain.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveTest extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final XboxController xboxController;
    private double vx =0.5;
    private double vy = 0;
    private double omega = 0;


    public DriveTest(SwerveDrive swerveDrive, XboxController xboxController) {
        this.swerveDrive = swerveDrive;
        this.xboxController = xboxController;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
              MathUtil.applyDeadband(vx, 0.05),
              MathUtil.applyDeadband(vy, 0.05),
              MathUtil.applyDeadband(omega, 0.05),
                false
        );
    }
}
