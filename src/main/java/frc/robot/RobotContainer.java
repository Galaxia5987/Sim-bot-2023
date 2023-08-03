package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.ArmXboxControl;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.command.XboxDrive;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Arm arm = Arm.getInstance();

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    private final JoystickButton lb = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rb = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

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
        swerveDrive.setDefaultCommand(new XboxDrive(swerveDrive, driverController));
        arm.setDefaultCommand(new ArmXboxControl(operatorController));
    }

    private void configureButtonBindings() {
        lb.onTrue(new InstantCommand(swerveDrive::resetGyro));
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
