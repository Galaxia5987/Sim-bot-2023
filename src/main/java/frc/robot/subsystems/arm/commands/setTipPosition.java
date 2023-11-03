package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmKinematics;

public class setTipPosition extends CommandBase {
    private Arm arm = Arm.getINSTANCE();
    private ArmKinematics armKinematics;
    private Translation2d endEffectorPosition;

    private setTipPosition(Arm arm, Translation2d endEffectorPosition) {
        this.armKinematics = new ArmKinematics(ArmConstants.SHOULDER_LENGTH, ArmConstants.ELBOW_LENGTH);
        this.endEffectorPosition = endEffectorPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setEndEffectorPosition(endEffectorPosition, armKinematics);
    }
}
