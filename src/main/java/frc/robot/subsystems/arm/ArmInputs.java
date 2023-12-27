package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmInputs implements LoggableInputs {

    public double shoulderAngle;
    public double elbowAngle;
    public double shoulderMotorPower;
    public double elbowMotorPower;
    public double shoulderSetpoint;
    public double elbowSetpoint;
    public double shoulderEncoderPosition;
    public double elbowEncoderPosition;
    public double shoulderVelocity;
    public double elbowVelocity;
    public double[] armPosition;
    public double[] inverseKinematicsSolution;
    public double[] feedforward;
    public double shoulderAcceleration;
    public double elbowAcceleration;
    public Rotation2d shoulderError;
    public Rotation2d elbowError;
    public double shoulderOutputVoltage;
    public double elbowOutputVoltage;
    public Translation2d finalSetpointAngles;
    public double ySetpoint;
    public double xSetpoint;
    public double shoulderAngleSetPoint = 0;
    double shoulderAppliedVoltage;
    double elbowAppliedVoltage;
    double shoulderAppliedCurrent;
    double elbowAppliedCurrent;
    double[] shoulderTipPose;
    double[] endEffectorPositionSetPoint;
    double elbowAngleSetpoint;
    double elbowAngleRelative;
    double elbowAngleAbsolute;
    Translation2d endEffectorPose;
    ArmIO.ControlMode shoulderControlMode;
    ArmIO.ControlMode elbowControlMode;


    
    public void toLog(LogTable table) {
        table.put("shoulderAngle", shoulderAngle);
        table.put("elbowAngle", elbowAngle);
        table.put("shoulderMotorPower", shoulderMotorPower);
        table.put("elbowMotorPower", elbowMotorPower);
        table.put("shoulderSetpoint", shoulderSetpoint);
        table.put("elbowSetpoint", elbowSetpoint);
        table.put("shoulderEncoderPosition", shoulderEncoderPosition);
        table.put("elbowEncoderPosition", elbowEncoderPosition);
        table.put("shoulderVelocity", shoulderVelocity);
        table.put("armPosition", armPosition);
        table.put("inverseKinematicsSolution", inverseKinematicsSolution);
        table.put("feedforward", feedforward);
        table.put("shoulderAcceleration", shoulderAcceleration);
        table.put("elbowAcceleration", elbowAcceleration);
        table.put("shoulderError", shoulderError);
        table.put("elbowError", elbowError);
        table.put("shoulderOutputVoltage", shoulderOutputVoltage);
        table.put("elbowOutputVoltage", elbowOutputVoltage);
        table.put("finalSetpointAngles", finalSetpointAngles);
        table.put("ySetpoint", ySetpoint);
        table.put("xSetpoint", xSetpoint);
        table.put("shoulderAppliedVoltage", shoulderAppliedVoltage);
        table.put("elbowAppliedVoltage", elbowAppliedVoltage);
        table.put("shoulderAppliedCurrent", shoulderAppliedCurrent);
        table.put("elbowAppliedCurrent", elbowAppliedCurrent);
        table.put("shoulderTipPose", shoulderTipPose);
        table.put("endEffectorPositionSetPoint", endEffectorPositionSetPoint);
        table.put("elbowAngleSetpoint", elbowAngleSetpoint);
        table.put("endEffectorPose", endEffectorPose);
        table.put("shoulderControlMode", shoulderControlMode);
        table.put("elbowControlMode", elbowControlMode);
    }

    @Override
    public void fromLog(LogTable table) {
        elbowMotorPower = table.get("elbowPower", elbowMotorPower);
        elbowMotorPower = table.get("shoulderPower", shoulderMotorPower);
        elbowAngle = table.get("elbowAngle", elbowAngle);
        shoulderAngle = table.get("shoulderAngle", shoulderAngle);
        elbowVelocity = table.get("elbowVelocity", elbowVelocity);
        shoulderVelocity = table.get("shoulderVelocity", shoulderVelocity);
        elbowAcceleration = table.get("elbowAcceleration", elbowAcceleration);
        shoulderAcceleration = table.get("shoulderAcceleration", shoulderAcceleration);
        elbowSetpoint = table.get("elbowSetPoint", elbowSetpoint);
        shoulderSetpoint = table.get("shoulderSetPoint", shoulderSetpoint);
        elbowEncoderPosition = table.get("elbowEncoderPosition", elbowEncoderPosition);
        shoulderEncoderPosition = table.get("shoulderEncoderPosition", shoulderEncoderPosition);
        armPosition = table.get("armPosition", armPosition);
        inverseKinematicsSolution = table.get("inverseKinematicsSolution", inverseKinematicsSolution);
        feedforward = table.get("feedforward", feedforward);
        shoulderOutputVoltage = table.get("shoulderOutputVoltage", shoulderOutputVoltage);
        elbowOutputVoltage = table.get("elbowOutputVoltage", elbowOutputVoltage);
        finalSetpointAngles = table.get("finalSetpointAngles", finalSetpointAngles);
        ySetpoint = table.get("ySetpoint", ySetpoint);
        xSetpoint = table.get("xSetpoint", xSetpoint);
        elbowAppliedVoltage = table.get("elbowAppliedVoltage", elbowAppliedVoltage);
        shoulderAppliedVoltage = table.get("shoulderAppliedVoltage", shoulderAppliedVoltage);
        elbowAppliedCurrent = table.get("elbowAppliedCurrent", elbowAppliedCurrent);
        shoulderAppliedCurrent = table.get("shoulderAppliedCurrent", shoulderAppliedCurrent);
        shoulderTipPose = table.get("shoulderTipPose", shoulderTipPose);
        endEffectorPositionSetPoint = table.get("endEffectorPositionSetPoint", endEffectorPositionSetPoint);
        elbowControlMode = table.get("elbowControlMode", elbowControlMode);
        shoulderControlMode = table.get("shoulderControlMode", shoulderControlMode);

    }
}
