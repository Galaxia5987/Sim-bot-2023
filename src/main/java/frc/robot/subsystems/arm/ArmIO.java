package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

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

    enum ControlMode {
        PRECENT_OUTPUT,
        POSITION
    }
}
