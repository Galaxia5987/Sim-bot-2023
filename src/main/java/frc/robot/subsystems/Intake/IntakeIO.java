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
        double powerCurrent = 0;
        double angleMotorCurrent = 0;
        double powerMotorPower = 0;
        double angleMotorAngle = 0;
        double angleMotorPower = 0;

    }
}
