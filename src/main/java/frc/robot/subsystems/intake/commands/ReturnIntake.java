package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class ReturnIntake extends SequentialCommandGroup {

    public ReturnIntake() {
        Intake intake = Intake.getInstance();

        addCommands(
                new Retract(Retract.Mode.UP),
                new InstantCommand(() -> intake.setPower(0))
        );
    }
}