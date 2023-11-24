package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.intake.Intake;

public class ArmKeyboardControl extends CommandBase {
    private final GenericHID controller = new GenericHID(0);
    private Arm arm = Arm.getINSTANCE();
    private Intake intake = Intake.getInstance();

    public ArmKeyboardControl() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(1)) {
            arm.setEndEffectorPosition(ArmConstants.FEEDER_POSITION, ArmIO.armKinematics);
        }
        if (controller.getRawButton(2)) {
            arm.setEndEffectorPosition(ArmConstants.STARTING_POSITION, ArmIO.armKinematics);
        }
    }
}
