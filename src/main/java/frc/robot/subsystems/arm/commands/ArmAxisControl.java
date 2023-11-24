package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmKinematics;

public class ArmAxisControl extends CommandBase {
    private final Arm arm = Arm.getINSTANCE();
    private ArmKinematics armKinematics;
    private final double Xvalue;
    private final double Yvalue;
    private Translation2d pose = new Translation2d(0, 0);

    public ArmAxisControl(double Xvalue, double Yvalue) {
        this.Xvalue = Xvalue;
        this.Yvalue = Yvalue;
        addRequirements(arm);
    }


    @Override
    public void initialize() {
        arm.setElbowP(0.02);
        pose = arm.getEndPosition();
    }

    @Override
    public void execute() {
        boolean passedMaximum = pose.getNorm() >= ArmConstants.SHOULDER_ARM_LENGTH + ArmConstants.ELBOW_ARM_LENGTH - 0.1;

        if (!passedMaximum) {
            pose = pose.plus(new Translation2d(Xvalue, Yvalue));
            arm.setEndEffectorPosition(pose);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setElbowP(ArmConstants.elbowP);
    }
}
