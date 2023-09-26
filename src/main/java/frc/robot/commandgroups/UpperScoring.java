package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.commands.ArmWithStateMachine;
import frc.robot.subsystems.arm.commands.SetArmsPositionAngular;
import frc.robot.subsystems.leds.Leds;

public class UpperScoring extends SequentialCommandGroup {

    public UpperScoring() {
        Arm arm = Arm.getInstance();
        addCommands(
                new ConditionalCommand(
                        new ArmWithStateMachine(ArmPosition.TOP_SCORING),
                        new ArmWithStateMachine(ArmPosition.TOP_SCORING),
                        Leds.getInstance()::inConeMode
                ),
                new RunCommand(() -> arm.setShoulderJointPower(-0.05), arm).withTimeout(0.5)
        );
    }
}
