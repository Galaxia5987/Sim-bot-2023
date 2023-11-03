package frc.robot.autonomous.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.autonomous.FollowPath;
import frc.robot.subsystems.drivetrain.SwerveConstants;
import frc.robot.subsystems.gripper.Gripper;

public class BumperConeHighCubeEngage extends AutoFunctions {

    public BumperConeHighCubeEngage() {
        Gripper gripper = Gripper.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("BumperConeCubeHigh 1", new PathConstraints(SwerveConstants.MAX_VELOCITY_AUTO, SwerveConstants.MAX_ACCELERATION_AUTO));

        addCommands(
                autoBegin(trajectory),

                bumperTakeCube(),

                FollowPath.loadTrajectory("BumperConeHighTakeCube engage"),

                engage(false, false)
                        .alongWith(new ReturnIntake()
                                .andThen(new InstantCommand(gripper::close)))
        );
    }
}
