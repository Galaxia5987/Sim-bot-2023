package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drive;

public class TestDriveResponseSim extends CommandBase {

    private final Drive drive = Drive.getInstance();

    private final Timer timer = new Timer();

    private final double frequency;

    public TestDriveResponseSim(double frequency) {
        this.frequency = frequency;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1 / frequency)) {
            drive.drive(
                    Math.random(),
                    Math.random(),
                    Math.random(),
                    0,
                    true
            );
            timer.reset();
        }
    }
}
