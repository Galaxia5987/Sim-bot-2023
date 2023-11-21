package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;

public class KeyBoardControl extends CommandBase {
    private final GenericHID controller = new GenericHID(0);
    private Arm arm = Arm.getINSTANCE();

    public KeyBoardControl() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(1)) {
            arm.setEndEffectorPosition(ArmConstants.FEEDER_POSITION, ArmIO.armKenematics);
        }
        if (controller.getRawButton(2)) {
            arm.setEndEffectorPosition(ArmConstants.STARTING_POSITION, ArmIO.armKenematics);
        }
        if (controller.getRawButton(3)) {
            arm.setShoulderPower(0.001);
        }
        if (controller.getRawButton(4)) {
            arm.setShoulderPower(-0.001);
        }
    }
}
