package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.SwerveDrive;
//import frc.robot.subsystems.drivetrain.command.JoystickDrive;
import frc.robot.subsystems.drivetrain.command.JoystickDrive;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();

    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);

    private final XboxController xboxController = new XboxController(0);
    private final JoystickButton lb = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rb = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftJoystickTrigger = new JoystickButton(leftJoystick, Joystick.ButtonType.kTrigger.value);

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
//        swerveDrive.setDefaultCommand(new XboxDrive(swerveDrive, xboxController));
        swerveDrive.setDefaultCommand(new JoystickDrive(swerveDrive, leftJoystick, rightJoystick));
    }

    private void configureButtonBindings() {
        leftJoystickTrigger.onTrue(new InstantCommand(swerveDrive::resetGyro));

        lb.onTrue(new InstantCommand(swerveDrive::resetGyro));
        rb.onTrue(new InstantCommand(swerveDrive::resetPose));
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
