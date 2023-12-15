package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import frc.robot.Ports;

import javax.swing.text.Position;


public class IntakeIOReal implements IntakeIO {


    private final CANSparkMax spinMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(21);
    private final TalonFXConfigurator angleConfigurator;
    private TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public IntakeIOReal() {
        angleConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.ANGLE_GEAR_RATIO;
        angleConfiguration.Feedback.RotorToSensorRatio = 1;

        angleConfigurator = angleMotor.getConfigurator();
        angleConfigurator.apply(angleConfiguration.Slot0);

        spinMotor.restoreFactoryDefaults();
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        spinMotor.setInverted(Ports.Intake.POWER_INVERTED);
        for (int i = 1; i <= 6; i++) {
            spinMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.fromId(i), 50000);
        }
        spinMotor.burnFlash();
        System.out.println(angleMotor);
    }


    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.set(power);
    }

    @Override
    public void setAngleMotorAngle(Rotation2d angle) {
        angleMotor.setControl(positionRequest.withPosition(angle.getRotations()).withEnableFOC(true));
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.setControl(dutyCycleRequest.withOutput(power).withEnableFOC(true));
    }

    @Override
    public void resetEncoder(double angle) {
    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        var powerDistribution = new PowerDistribution(0, PowerDistribution.ModuleType.kRev);

        inputs.angleMotorAppliedVoltage = angleMotor.getMotorVoltage().getValue();
        inputs.angleMotorAppliedCurrent = angleMotor.getSupplyCurrent().getValue();
        inputs.angleMotorPower = inputs.angleMotorAppliedVoltage / powerDistribution.getVoltage();
        inputs.angleMotorAngle = Rotation2d.fromRotations(angleMotor.getPosition().getValue());
        inputs.angleMotorVelocity = Rotation2d.fromRotations(angleMotor.getVelocity().getValue());
        inputs.spinMotorPower = spinMotor.getAppliedOutput();
        inputs.spinMotorAppliedCurrent = spinMotor.getOutputCurrent();
        inputs.spinMotorAppliedVoltage = spinMotor.getVoltageCompensationNominalVoltage();
    }
}
