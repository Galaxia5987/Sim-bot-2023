package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void updateInputs(IntakeInputs inputs){
    }

    default void setAngleMotorVelocity(double velocity){
    }

    default void setSpinMotorVelocity(double velocity){
    }

    default void setAngle(double angle){
    }

    @AutoLog
    class IntakeInputs{
        public double angle = 0;
        public double angleSetpoint = 0;
        public double angleMotorVelocity = 0;
        public double angleMotorVelocitySetpoint = 0;
        public double spinMotorVelocity = 0;
        public double spinMotorVelocitySetpoint = 0;

    }
}
