package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class HoldIntakeInPlace extends CommandBase {
    private final Intake intake = Intake.getInstance();

    public HoldIntakeInPlace() {
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (intake.getAngle() > -5) {
            intake.setPower(0);
        }
        intake.setAngle(IntakeConstants.ANGLE_UP);
    }
}
