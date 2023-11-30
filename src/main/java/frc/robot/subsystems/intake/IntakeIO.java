package frc.robot.subsystems.intake;

public interface IntakeIO {
    void updateInputs(IntakeLoggedInputs inputs);

    void setSpinMotorPower(double power);

    void setAngleMotorAngle(double angle);

    void setAngleMotorPower(double power);

    void resetEncoder(double angle);
}
