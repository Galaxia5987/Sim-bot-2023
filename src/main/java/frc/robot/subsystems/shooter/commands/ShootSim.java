package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.shooter.ShooterIOSim;

public class ShootSim extends CommandBase {
    private final ShooterIOSim shooter = ShooterIOSim.getInstance();

    private final Timer timer = new Timer();

    private final double frequency;

    public ShootSim(double frequency) {
        this.frequency = frequency;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(1 / frequency)) {
            shooter.setVelocity(Math.random() * 5000 + 1);
            timer.reset();
        }
    }
}
