package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;

public interface ArmIO {
    double currentShoulderAngle = 0;
    double currentElbowAngle = 0;

    ArmKinematics armKinematics = new ArmKinematics(ArmConstants.SHOULDER_LENGTH, ArmConstants.ELBOW_LENGTH);

    default void setShoulderPower(double power) {
    }

    default void setElbowPower(double power) {
    }

    default void setShoulderAngle(double angle) {
    }

    default void setElbowAngle(double angle) {
    }

    default void setEndEffectorPosition(Translation2d position, ArmKinematics armKinematics) {
    }

    default void setElbowP(double kP) {

    }

    default void updateInputs() {
    }

    String getSubsystemName();

    void updateInputs(ArmInputs inputs);

    enum ControlMode {
        PRECENT_OUTPUT,
        POSITION
    }
}
