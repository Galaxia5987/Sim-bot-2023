package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class KeyBoardControl extends CommandBase {
    private final GenericHID controller = new GenericHID(0);
    private Arm arm = Arm.getINSTANCE();

    public KeyBoardControl() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(1)) {
            arm.setElbowPower(0.001);
        }
        if (controller.getRawButton(2)) {
            arm.setElbowPower(-0.001);

        }
        if (controller.getRawButton(3)) {
            arm.setShoulderPower(0.001);
        }
        if (controller.getRawButton(4)) {
            arm.setShoulderPower(-0.001);
        }
    }
}
