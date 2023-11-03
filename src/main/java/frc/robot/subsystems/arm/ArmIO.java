package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    static ArmKinematics armKenematics = new ArmKinematics(ArmConstants.SHOULDER_LENGTH, ArmConstants.ELBOW_LENGTH);

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

    default void updateInputs() {
    }

    @AutoLog
    class ArmInputs {

        double shoulderAppliedVoltage = 0;
        double elbowAppliedVoltage = 0;
        double shoulderAppliedCurrent = 0;
        double elbowAppliedCurrent = 0;
        double[] shoulderTipPose = new double[2];
        double[] endEffectorPosition = new double[2];
        double shoulderAngle = 0;
        double elbowAngleRelative = 0;
        double elbowAngleAbsolute = 0;


    }
}
