package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.swerve.SwerveDrive;
import frc.robot.common.swerve.commands.XboxDrive;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final SwerveDrive drive = SwerveDrive.getInstance();
    private final XboxController xboxController = new XboxController(0);

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
                new XboxDrive(drive, xboxController)
        );
    }

    private void configureButtonBindings() {
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
