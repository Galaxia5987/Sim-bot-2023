package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeKeyboardControl extends Command {
    private final GenericHID controller = new GenericHID(0);
    private final Intake intake = Intake.getInstance();

    public IntakeKeyboardControl() {
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(3)) {
            intake.setAngleMotorAngle(IntakeConstants.ANGLE_UP);
        }
        if (controller.getRawButton(4)) {
            intake.setAngleMotorAngle(IntakeConstants.ANGLE_DOWN);
        }
    }
}