package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.autonomous.FollowPath;
import frc.robot.commandgroups.PickUpCubeAuto;
import frc.robot.commandgroups.ReturnIntake;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.commands.ArmWithStateMachine;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.gripper.Gripper;

/**
 * This class contains all parts of the path FeederConeCubeHigh.
 * <p>
 * In this path the robot places a cone in the grid that is closest to the feeder,
 * goes to take a cube (the one closest to the feeder) and returns to place it.
 */
public class FeederConeCubeHighCube extends AutoFunctions {
    public FeederConeCubeHighCube() {
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("FeederConeCubeHigh 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                autoBegin(trajectory),

                feederTakeCube(),

                FollowPath.loadTrajectory("FeederConeCubeHigh 2")
                        .alongWith(new ReturnIntake().andThen(new InstantCommand(gripper::close, gripper))
                                .andThen(new ArmWithStateMachine(ArmPosition.FEEDER).withTimeout(1.0))),

                autoUpperScoring(false),

                FollowPath.loadTrajectory("FeederConeCubeHigh 3").alongWith(new PickUpCubeAuto().withTimeout(5))
        );
    }
}