package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drive;

public class KeyboardDriveSim extends CommandBase {

    private final Drive drive = Drive.getInstance();

    private final GenericHID controller = new GenericHID(0);

    public KeyboardDriveSim() {
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xOutput = -controller.getRawAxis(1);
        double yOutput = -controller.getRawAxis(0);
        double omegaOutput = controller.getRawAxis(2);

//        System.out.println(controller.isConnected());
//        System.out.println("xOutput: " + xOutput);
//        System.out.println("yOutput: " + yOutput);
//        System.out.println("omegaOutput: " + omegaOutput);
//        System.out.println();

        drive.drive(
                xOutput,
                yOutput,
                omegaOutput,
                0.01,
                true
        );
    }
}
