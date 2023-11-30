package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.intake.Intake;

public class ArmKeyboardControl extends Command {
    private final GenericHID controller = new GenericHID(0);
    private final Arm arm = Arm.getINSTANCE();
    private final Intake intake = Intake.getInstance();

    public ArmKeyboardControl() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(1)) {
            arm.setEndEffectorPosition(ArmConstants.FEEDER_POSITION);
        }
        if (controller.getRawButton(2)) {
            arm.setEndEffectorPosition(ArmConstants.STARTING_POSITION);
        }
    }
}
