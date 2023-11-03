package frc.robot.autonomous.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoFunctions;
import frc.robot.autonomous.FollowPath;

public class BumperCone2Cubes extends AutoFunctions {
    public BumperCone2Cubes() {
        addCommands(
                new BumperConeCubeHighCube(),
                FollowPath.loadTrajectory("BumperConeCubeHigh 4").alongWith(new ReturnIntake()
                        .andThen(new InstantCommand(gripper::close, gripper))
                        .andThen(autoMidScoring(false))),
                new InstantCommand(gripper::open, gripper)
        );
    }
}
