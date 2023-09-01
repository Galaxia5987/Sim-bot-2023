package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.command.DriveTest;
import frc.robot.subsystems.drivetrain.command.DriveTest2;
import frc.robot.subsystems.drivetrain.command.XboxDrive;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    private final XboxController xboxController = new XboxController(0);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton a = new JoystickButton(xboxController, XboxController.Button.kA.value);
    private final SwerveModuleState swerveModuleStates1 = new SwerveModuleState(1, new Rotation2d(0,0));
    private final SwerveModuleState swerveModuleStates2 = new SwerveModuleState(1, new Rotation2d(0,0));
    private final SwerveModuleState swerveModuleStates3 = new SwerveModuleState(1, new Rotation2d(0,0));
    private final SwerveModuleState swerveModuleStates4 = new SwerveModuleState(1, new Rotation2d(0,0));
    private final SwerveModuleState[] swerveModuleStates = new SwerveModuleState[]{swerveModuleStates1,swerveModuleStates2,swerveModuleStates3,swerveModuleStates4};


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    public static RobotContainer getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotContainer();
        }
        return INSTANCE;
    }

    private void configureDefaultCommands() {
//        swerveDrive.setDefaultCommand(new DriveTest(swerveDrive, xboxController));
    }

    private void configureButtonBindings() {
        lb.onTrue(new InstantCommand(swerveDrive::resetGyro));
        rb.onTrue(new InstantCommand(swerveDrive::resetPose));
        a.whileTrue(new DriveTest2(swerveDrive));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
