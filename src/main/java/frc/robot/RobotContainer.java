package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.commands.KeyboardDriveSim;
import frc.robot.subsystems.drivetrain.commands.XboxDrive;

public class RobotContainer {
    private static RobotContainer INSTANCE = null;

    private final Drive drive = Drive.getInstance();

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
                new KeyboardDriveSim()
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
        return new InstantCommand(() -> drive.resetOdometry(new Pose2d(), 0))
                .andThen(new PPSwerveControllerCommand(
                        PathPlanner.loadPath("Test", 5, 3),
                        drive::getCurrentPose,
                        new PIDController(7, 0, 0),
                        new PIDController(7, 0, 0),
                        new PIDController(10, 0, 0),
                        (speeds) -> drive.drive(speeds, false),
                        false,
                        drive
                ));
    }

}
