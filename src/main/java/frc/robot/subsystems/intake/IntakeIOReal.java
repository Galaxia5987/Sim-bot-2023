package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;
import frc.robot.Ports;
import utils.Utils;
import utils.units.UnitModel;


public class IntakeIOReal implements IntakeIO {
    private final CANSparkMax spinMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotorPhx = new TalonFX(0);

    private final TalonFXConfigurator angleConfigurator;
    private final TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();
    private final UnitModel unitModel = new UnitModel(IntakeConstants.TICKS_PER_DEGREE);

    public IntakeIOReal() {
        angleConfigurator = angleMotorPhx.getConfigurator();
        angleConfiguration.Slot0.kP = IntakeConstants.kP;
        angleConfiguration.Slot0.kI = IntakeConstants.kI;
        angleConfiguration.Slot0.kD = IntakeConstants.kD;

        spinMotor.restoreFactoryDefaults();

        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        spinMotor.setInverted(Ports.Intake.POWER_INVERTED);
        for (int i = 1; i <= 6; i++) {
            spinMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.fromId(i), 50000);
        }
        spinMotor.burnFlash();

angleConfiguration.
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.set(power);
    }

    @Override
    public void setAngleMotorAngle(double angle) {



        angleMotorPhx.setControl(new PositionVoltage(Utils.));

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void resetEncoder(double angle) {
        angleMotor.setSelectedSensorPosition(
                unitModel.toTicks(angle)
        );
    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        inputs.spinMotorPower = spinMotor.getAppliedOutput();
        inputs.angleMotorAngle = unitModel.toUnits(angleMotor.getSelectedSensorPosition());
        inputs.angleMotorVelocity = unitModel.toVelocity(angleMotor.getSelectedSensorVelocity());
        inputs.angleMotorcurrent = angleMotor.getSupplyCurrent();
        inputs.angleMotorPower = angleMotor.getMotorOutputPercent();

        angleConfigurator.apply(angleConfiguration.Slot0);

    }
}
