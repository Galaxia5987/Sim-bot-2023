package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.commandgroups.GetArmIntoRobot;
import frc.robot.commandgroups.PickUpCubeAuto;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveTillPitch;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.utils.controllers.DieterController;

public class MiddleConeHighCubeEngage extends AutoFunctions {
    private final DieterController yawController = new DieterController(3, 0, 0, 0);

    public MiddleConeHighCubeEngage() { //TODO: check if it works
        SwerveDrive swerveDrive = SwerveDrive.getInstance();
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("MiddleConeHighCubeEngage 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                autoBegin(trajectory),

                new ReturnArm().withTimeout(1),

                new DriveTillPitch(8.5, 1.5),

                new RunCommand(() -> swerveDrive.drive(
                        1.5,
                        0,
                        yawController.calculate(swerveDrive.getYaw(), 0),
                        true
                ), swerveDrive)
                        .alongWith(new PickUpCubeAuto())
                        .withTimeout(1.9),

                engage(false, false)
                        .alongWith(new ReturnIntake()
                                .andThen(new InstantCommand(gripper::close)))
        );
    }
}
