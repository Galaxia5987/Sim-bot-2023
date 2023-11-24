package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Utils;
import frc.robot.utils.math.differential.BooleanTrigger;

public class ArmXboxControl extends CommandBase {
    private final Arm arm = Arm.getINSTANCE();
    private final XboxController xboxController;
    private final BooleanTrigger holdTrigger = new BooleanTrigger(false, false);
    private double shoulderHoldAngle;
    private double elbowHoldAngle;

    public ArmXboxControl(XboxController xboxController) {
        this.xboxController = xboxController;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        double powerSho = MathUtil.applyDeadband(-xboxController.getLeftY(), 0.2);
        double powerEl = MathUtil.applyDeadband(-xboxController.getRightY(), 0.2);

        boolean joysticksZero = Utils.epsilonEquals(powerSho, 0) && Utils.epsilonEquals(powerEl, 0);
        holdTrigger.update(joysticksZero);

        if (holdTrigger.triggered() || Robot.justEnabled() || arm.changedToDefaultCommand()) {
            shoulderHoldAngle = arm.getShoulderAngle();
            elbowHoldAngle = arm.getElbowAngleRelative();
        }
        if (joysticksZero) {
            arm.setShoulderAngle(shoulderHoldAngle);
            arm.setElbowAngleRelative(elbowHoldAngle);
        } else {
            arm.setShoulderPower(0.1 * powerSho);
            arm.setElbowPower(0.1 * powerEl);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
