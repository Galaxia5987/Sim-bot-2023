package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drive;

public class SonyDrive extends CommandBase {

    private final Drive drive = Drive.getInstance();

    private final PS4Controller ps4Controller;

    public SonyDrive(PS4Controller ps4Controller) {
        this.ps4Controller = ps4Controller;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
                -ps4Controller.getLeftY(),
                -ps4Controller.getLeftX(),
                -ps4Controller.getRightX(),
                0.15,
                true
        );
    }

}
