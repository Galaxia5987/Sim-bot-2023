package frc.robot.commandgroups.bits;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import swerve.SwerveDrive;

import java.util.Arrays;

public class ZeroPositionSwerve extends Command {
    private final SwerveDrive swerve = SwerveDrive.getInstance(Robot.isReal());
    private final Timer timer = new Timer();
    private final SwerveModuleState[] zeroStates = new SwerveModuleState[4];

    public ZeroPositionSwerve() {
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        Arrays.fill(zeroStates, new SwerveModuleState());
    }


    @Override
    public void execute() {
        swerve.setModuleStates(zeroStates);
    }


    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.setModuleStates(new SwerveModuleState[4]);
    }
}

