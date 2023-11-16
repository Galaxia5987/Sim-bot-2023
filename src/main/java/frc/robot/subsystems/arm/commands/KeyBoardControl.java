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
            arm.setShoulderAngle(Math.PI / 2);
        }
        System.out.println(controller.getRawButton(1));
    }
}
