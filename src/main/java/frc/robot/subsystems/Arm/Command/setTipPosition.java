package frc.robot.subsystems.Arm.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;

public class setTipPosition extends CommandBase {
    private final Arm arm = Arm.getInstance();
    public double xPose;
    public double yPose;

    public setTipPosition(double xPose, double yPose){
        this.xPose = xPose;
        this.yPose = yPose;
    }
    @Override
    public void execute() {
        arm.setArmPosition(xPose, yPose);
    }
}
