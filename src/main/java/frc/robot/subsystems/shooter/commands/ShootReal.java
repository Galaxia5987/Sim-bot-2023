package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterIOReal;

import java.util.function.DoubleSupplier;

public class ShootReal extends CommandBase {

    private final ShooterIOReal shooter = ShooterIOReal.getInstance();
    private final DoubleSupplier velocity;

    public ShootReal(DoubleSupplier velocity) {
        this.velocity = velocity;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocity.getAsDouble());
    }
}
