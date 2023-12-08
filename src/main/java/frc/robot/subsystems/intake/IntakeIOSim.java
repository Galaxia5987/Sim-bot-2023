package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import utils.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private final TalonFXSim angleMotorSim;
    private final TalonFXSim powerMotorSim;
    private final TalonFXConfiguration angleConfiguration;

    public IntakeIOSim() {
        angleConfiguration = new TalonFXConfiguration();

        angleMotorSim = new TalonFXSim(0, IntakeConstants.ANGLE_GEAR_RATIO, 1);
        powerMotorSim = new TalonFXSim(1, IntakeConstants.SPIN_GEAR_RATIO, 1);

        angleConfiguration.Slot0.kP = IntakeConstants.kP_SIM;
        angleConfiguration.Slot0.kI = IntakeConstants.kI_SIM;
        angleConfiguration.Slot0.kD = IntakeConstants.kD_SIM;
        angleMotorSim.configure(angleConfiguration);
    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        inputs.angleMotorPower = angleMotorSim.getAppliedVoltage() / 12;
        inputs.angleMotorcurrent = angleMotorSim.getAppliedCurrent();
        inputs.angleMotorVelocity = angleMotorSim.getRotorVelocity();
        inputs.angleMotorAngle = angleMotorSim.getRotorPosition() * Math.PI * 2 % Math.PI * 2;
        inputs.spinMotorCurrent = powerMotorSim.getAppliedVoltage();
        inputs.spinMotorPower = powerMotorSim.getAppliedVoltage() / 12;
    }

    @Override
    public void setSpinMotorPower(double power) {
        var motorRequest = new DutyCycleOut(power);
        powerMotorSim.setControl(motorRequest);
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        var motorRequest = new PositionVoltage(Units.radiansToRotations(angle));
        angleMotorSim.setControl(motorRequest);
    }

    @Override
    public void setAngleMotorPower(double power) {
        var motorRequest = new DutyCycleOut(power);
        angleMotorSim.setControl(motorRequest);
    }


    @Override
    public void resetEncoder(double angle) {

    }
}
