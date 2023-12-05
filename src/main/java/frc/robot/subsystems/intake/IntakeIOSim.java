package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class IntakeIOSim implements IntakeIO {
    private final TalonFXSimState angleMotorPhx;
    private final TalonFX talonFXAngle;
    private final TalonFX talonFXPower;
    private final TalonFXSimState powerMotorPhx;
    private final TalonFXConfigurator angleConfigurator;
    private final TalonFXConfigurator powerConfigurator;
    private final TalonFXConfiguration angleConfiguration;

    public IntakeIOSim() {
        angleConfiguration = new TalonFXConfiguration();

        talonFXAngle = new TalonFX(0);
        talonFXPower = new TalonFX(1);

        angleMotorPhx = new TalonFXSimState(talonFXAngle);
        powerMotorPhx = new TalonFXSimState(talonFXPower);

        angleConfigurator = talonFXAngle.getConfigurator();
        powerConfigurator = talonFXPower.getConfigurator();

        angleConfiguration.Slot0.kP = IntakeConstants.kP_SIM;
        angleConfiguration.Slot0.kI = IntakeConstants.kI_SIM;
        angleConfiguration.Slot0.kD = IntakeConstants.kD_SIM;

        angleConfigurator.apply(angleConfiguration.Slot0);

    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        inputs.angleMotorPower = angleMotorPhx.getMotorVoltage() / 12;
        inputs.angleMotorcurrent = angleMotorPhx.getSupplyCurrent();
        inputs.angleMotorVelocity = talonFXAngle.getVelocity().getValue();
        inputs.angleMotorAngle = talonFXAngle.getRotorPosition().getValue() * Math.PI * 2 % Math.PI * 2;
        inputs.spinMotorCurrent = powerMotorPhx.getSupplyCurrent();
        inputs.spinMotorPower = powerMotorPhx.getMotorVoltage() / 12;
    }

    @Override
    public void setSpinMotorPower(double power) {
        powerMotorPhx.setSupplyVoltage(power * 12);
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        angleMotorPhx.setRawRotorPosition(angle / 2 * Math.PI);
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotorPhx.setSupplyVoltage(power * 12);
    }


    @Override
    public void resetEncoder(double angle) {

    }
}
