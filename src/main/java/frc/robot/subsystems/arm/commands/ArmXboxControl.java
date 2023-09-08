package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.Utils;
import frc.robot.utils.math.differential.BooleanTrigger;

public class ArmXboxControl extends CommandBase {
    private final Arm arm = Arm.getInstance();
    private final XboxController xboxController;

    private double shoulderHoldAngle;
    private double elbowHoldAngle;

    private final BooleanTrigger holdTrigger = new BooleanTrigger(false, false);

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
            shoulderHoldAngle = arm.getShoulderJointAngle().getDegrees();
            elbowHoldAngle = arm.getElbowJointAngle().getDegrees();
        }
        if (joysticksZero) {
            arm.setShoulderJointAngle(shoulderHoldAngle, 1);
            arm.setElbowJointAngle(elbowHoldAngle, 1);
        } else {
            arm.setShoulderJointPower(0.3 * powerSho);
            arm.setElbowJointPower(0.3 * powerEl);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
