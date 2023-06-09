package frc.robot.subsystems.Intake.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;

public class Feed extends CommandBase {
    Intake intake;
    double intakePower;

    public Feed(Intake intake, double intakePower) {
        this.intakePower = intakePower;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAngleMotor(IntakeConstants.FEED_ANGLE);
    }

    @Override
    public void execute() {
        intake.setPowerMotor(intakePower);
    }

    //TODO: add a default command for holding the Intake up.
}
