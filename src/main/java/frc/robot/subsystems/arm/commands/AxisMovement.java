package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmKinematics;

public class AxisMovement extends CommandBase {

    private final XboxController xboxController;
    private ArmKinematics armKinematics;
    private Arm arm = Arm.getINSTANCE();
    private Translation2d pose;
    private double xMultiplier;
    private double yMultiplier;
    private Translation2d startingPosition;

    public AxisMovement(Translation2d startingPosition, double xMultiplier, double yMultiplier) {
        this.xboxController = new XboxController(0);
        this.armKinematics = new ArmKinematics(ArmConstants.SHOULDER_LENGTH, ArmConstants.ELBOW_LENGTH);
        this.xMultiplier = xMultiplier;
        this.yMultiplier = yMultiplier;
    }

    @Override
    public void initialize() {
        arm.setEndEffectorPosition(startingPosition, armKinematics);
    }

    @Override
    public void execute() {
        pose = pose.plus(new Translation2d(xboxController.getLeftX()*xMultiplier,xboxController.getRightY()*yMultiplier));
        arm.setEndEffectorPosition(pose, armKinematics);
    }
}
