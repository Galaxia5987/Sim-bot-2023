package frc.robot.subsystems.Iitake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void setMotorPower(double power){

    }
    default void setAngleMotor(double desiredAngle){

    }
    default void updateInputs(IntakeInputs inputs){

    }

    @AutoLog
    class IntakeInputs{
        double motorPower = 0;
        double motorAngle = 0;
    }
}
