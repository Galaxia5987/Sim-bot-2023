package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.common.swerve.SwerveDrive;
import frc.robot.common.swerve.commands.JoystickDrive;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive drive = SwerveDrive.getInstance();
    private final Vision vision = Vision.getInstance();
    private final XboxController xboxController = new XboxController(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);

    private final JoystickButton leftJoystickTrigger = new JoystickButton(leftJoystick, 1);
    private final JoystickButton rightJoystickTrigger = new JoystickButton(rightJoystick, 1);

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
        drive.setDefaultCommand(
                new JoystickDrive(drive, leftJoystick, rightJoystick)
        );
    }

    private void configureButtonBindings() {
        leftJoystickTrigger.onTrue(new InstantCommand(drive::resetGyro));
        rightJoystickTrigger.onTrue(new InstantCommand(RobotState.getInstance()::reset));
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
