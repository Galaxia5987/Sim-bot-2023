package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drive;

public class XboxDrive extends CommandBase {

    private final Drive drive = Drive.getInstance();

    private final XboxController xboxController;

    public XboxDrive(XboxController xboxController) {
        this.xboxController = xboxController;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
                -xboxController.getLeftY(),
                -xboxController.getLeftX(),
                -xboxController.getRightX(),
                0.15,
                true
        );
    }
}
