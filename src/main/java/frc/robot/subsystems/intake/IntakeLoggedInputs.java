package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double spinMotorPower;
    public double setpointSpinMotorPower;
    public double setpointAngleMotorPower;
    public double setpointAngleMotorAngle;
    public double angleMotorPower;
    public double angleMotorAngle;
    public double angleMotorcurrent;
    public double spinMotorCurrent;
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
        table.put("angleMotorCurrent", angleMotorcurrent);
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
