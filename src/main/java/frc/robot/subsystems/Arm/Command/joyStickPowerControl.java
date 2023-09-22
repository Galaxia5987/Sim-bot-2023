package frc.robot.subsystems.Arm.Command;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.utils.Utils;

public class joyStickPowerControl extends CommandBase{
private XboxController xboxController = new XboxController(0);
private Arm arm = Arm.getInstance();
    @Override
    public void execute() {
        arm.setShoulderPower( xboxController.getLeftX());
        arm.setElbowPower(xboxController.getRightX());
    }

}
