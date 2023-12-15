package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLoggedInputs implements LoggableInputs {
    public double setpointSpinMotorPower;
    public double setpointAngleMotorPower;
    public Rotation2d setpointAngleMotorAngle = new Rotation2d();
    public double angleMotorAppliedVoltage;
    public double angleMotorAppliedCurrent;
    public double angleMotorPower;
    public Rotation2d angleMotorAngle = new Rotation2d();
    public double spinMotorPower;
    public double spinMotorAppliedCurrent;
    public double spinMotorAppliedVoltage;
    public Rotation2d angleMotorVelocity = new Rotation2d();


    /**
     * Implement the variables inside the table.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("spinMotorPower", spinMotorPower);
        table.put("setpointSpinMotorPower", setpointSpinMotorPower);
        table.put("setpointAngleMotorPower", setpointAngleMotorPower);
        table.put("setpointAngleMotorAngle", setpointAngleMotorAngle.getDegrees());

        table.put("angleMotorAngle", angleMotorAngle.getDegrees());
        table.put("angleMotorVelocity", angleMotorVelocity.getDegrees());
        table.put("angleMotorCurrent", angleMotorAppliedCurrent);
        table.put("angleMotorPower", angleMotorPower);
    }

    /**
     * Update the variables value from the logger.
     */
    @Override
    public void fromLog(LogTable table) {
        spinMotorPower = table.get("power", spinMotorPower);
        angleMotorAngle = table.get("angle", angleMotorAngle);
        angleMotorVelocity = table.get("velocity", angleMotorVelocity);
    }
}
