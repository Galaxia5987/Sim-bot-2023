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

    @AutoLog
    class ArmInputs {

        public double shoulderAngleSetPoint = 0;
        double shoulderAppliedVoltage = 0;
        double elbowAppliedVoltage = 0;
        double shoulderAppliedCurrent = 0;
        double elbowAppliedCurrent = 0;
        double[] shoulderTipPose = new double[2];
        double[] endEffectorPositionSetPoint = new double[2];
        double shoulderAngle = 0;
        double elbowAngleSetpoint = 0;
        double elbowAngleRelative = 0;
        double elbowAngleAbsolute = 0;
        Translation2d endEffectorPose;
        ControlMode shoulderControlMode = ControlMode.PRECENT_OUTPUT;
        ControlMode elbowControlMode = ControlMode.PRECENT_OUTPUT;
    }
}
