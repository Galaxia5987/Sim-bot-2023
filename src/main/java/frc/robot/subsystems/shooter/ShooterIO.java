package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.ShooterIO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    default void updateInput(ShooterInputs inputs){}

    default void setVelocity(double velocity) {}

    double getVelocity();

    double getSetpoint();

    default boolean atSetPoint(double tolerance){
        return false;
    }

    @AutoLog
    class ShooterInputs {
        public double velocity = 0;
        public double setpoint = 0;
    }
}
