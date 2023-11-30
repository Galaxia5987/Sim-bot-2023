package frc.robot.subsystems.intake.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class XboxWristControl extends Command {
    private final Intake intake = Intake.getInstance();
    private final XboxController xboxController;

    public XboxWristControl(XboxController xboxController) {
        this.xboxController = xboxController;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        double power = MathUtil.applyDeadband(-xboxController.getLeftY(), 0.2);
        intake.setAnglePower(power);
        intake.setSpinMotorPower(xboxController.getRightTriggerAxis());
    }
}
