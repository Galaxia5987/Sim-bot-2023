package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;

public class PickUpCubeAuto extends ParallelCommandGroup {

    public PickUpCubeAuto() {
        Arm arm = Arm.getINSTANCE();
        Gripper gripper = Gripper.getInstance();
        Intake intake = Intake.getInstance();

        addCommands(
                new GetArmIntoRobot(),
                new WaitUntilCommand(arm::armIsInRobot).andThen(new InstantCommand(gripper::open, gripper))
                        .andThen(new Feed(false))
        );
    }
}

