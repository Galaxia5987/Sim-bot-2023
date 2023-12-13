package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double setpointSpinMotorPower;
    public double setpointAngleMotorPower;
    public double setpointAngleMotorAngle;
    public double angleMotorAppliedVoltage;
    public double angleMotorAppliedCurrent;
    public double angleMotorPower;
    public double angleMotorAngle;
    public double spinMotorPower;
    public double spinMotorAppliedCurrent;
    public double spinMotorAppliedVoltage;
    public double angleMotorVelocity;


    /**
     * Implement the variables inside the table.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("spinMotorPower", spinMotorPower);
        table.put("setpointSpinMotorPower", setpointSpinMotorPower);
        table.put("setpointAngleMotorPower", setpointAngleMotorPower);
        table.put("setpointAngleMotorAngle", setpointAngleMotorAngle);
        table.put("angleMotorAngle", angleMotorAngle);
        table.put("angleMotorVelocity", angleMotorVelocity);
        table.put("angleMotorCurrent", angleMotorAppliedCurrent);
        table.put("angleMotorPower", angleMotorPower);
    }

    /**
     * Update the variables value from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        spinMotorPower = table.getDouble("power", spinMotorPower);
        angleMotorAngle = table.getDouble("angle", angleMotorAngle);
        angleMotorVelocity = table.getDouble("velocity", angleMotorVelocity);
    }
}
