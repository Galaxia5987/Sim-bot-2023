package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    default void setShoulderPower(double power) {
    }

    default void setElbowPower(double power) {
    }

    default void setShoulderAngle(double angle) {
    }

    default void setElbowAngle(double angle) {
    }

    default void updateInputs(){}

    @AutoLog
    class ArmInputs {

        double shoulderAppliedVoltage = 0;
        double elbowAppliedVoltage = 0;
        double shoulderAppliedCurrent= 0;
        double elbowAppliedCurrent = 0;

        double[] endEffectorPosition = new double[]{};
        double shoulderAngle = 0;
        double elbowAngleRelative = 0;
        double elbowAngleAbsolute = 0;


    }
}
