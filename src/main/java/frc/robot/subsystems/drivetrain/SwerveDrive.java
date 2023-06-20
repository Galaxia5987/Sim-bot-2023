package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] modules = new SwerveModule[4]; //FL, FR, RL, RR
    private final double[] offsets = {0, 0, 0, 0};

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            SwerveConstants.wheelPositions[0],
            SwerveConstants.wheelPositions[1],
            SwerveConstants.wheelPositions[2],
            SwerveConstants.wheelPositions[3]);

    public SwerveDrive(int[] driveMotorPorts, int[] angleMotorPorts, int[] encoderPorts) {
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(driveMotorPorts[i], angleMotorPorts[i], encoderPorts[i]);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setModuleState(desiredModuleStates[i]);
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the swerve
     *
     * @param xOutput     percentage of the x speed
     * @param yOutput     percentage of the y speed
     * @param omegaOutput percentage of the omega speed
     */
    public void drive(double xOutput, double yOutput, double omegaOutput) {

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                SwerveConstants.MAX_X_VELOCITY * xOutput,
                SwerveConstants.MAX_Y_VELOCITY * yOutput,
                SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput);

        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
    }
}
