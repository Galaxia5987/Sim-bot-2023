package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCubeAuto;
import frc.robot.commandgroups.ReturnArm;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.gripper.Gripper;

public class BumperConeCubeHighCube extends AutoFunctions {

    public BumperConeCubeHighCube() {
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("BumperConeCubeHigh 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                autoBegin(trajectory),

                bumperTakeCube(),

                FollowPath.loadTrajectory("BumperConeCubeHigh 2")
                        .alongWith(new ReturnIntake()
                                .andThen(new InstantCommand(gripper::close, gripper))
                                .andThen(new ReturnArm().withTimeout(0.65))),

                autoUpperScoring(false),

                FollowPath.loadTrajectory("BumperConeCubeHigh 3")
                        .alongWith(new PickUpCubeAuto().withTimeout(3.3))
        );
    }
}
