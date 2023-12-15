package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    void updateInputs(IntakeLoggedInputs inputs);

    void setSpinMotorPower(double power);

    void setAngleMotorAngle(Rotation2d angle);

    void setAngleMotorPower(double power);

    void resetEncoder(double angle);
}
