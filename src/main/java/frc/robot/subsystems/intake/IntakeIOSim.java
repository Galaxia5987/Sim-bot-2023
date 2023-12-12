package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import utils.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private final TalonFXSim angleMotorSim;
    private final TalonFXSim powerMotorSim;
    private final TalonFXConfiguration angleConfiguration;
    private final PIDController angleController = new PIDController(IntakeConstants.kP,IntakeConstants.kI,IntakeConstants.kD);

    public IntakeIOSim() {
        angleConfiguration = new TalonFXConfiguration();

        angleMotorSim = new TalonFXSim(1, IntakeConstants.ANGLE_GEAR_RATIO, 1);
        powerMotorSim = new TalonFXSim(1, IntakeConstants.SPIN_GEAR_RATIO, 1);
        angleMotorSim.setController(angleController);

        angleConfiguration.Slot0 = IntakeConstants.PIDGains;
//        angleMotorSim.configure(angleConfiguration);
    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        angleMotorSim.update(Timer.getFPGATimestamp());
        powerMotorSim.update(Timer.getFPGATimestamp());

        inputs.angleMotorPower = angleMotorSim.getAppliedVoltage() / 12;
        inputs.angleMotorAppliedCurrent = angleMotorSim.getAppliedCurrent();
        inputs.angleMotorAppliedVoltage = angleMotorSim.getAppliedVoltage();
        inputs.angleMotorVelocity = angleMotorSim.getRotorVelocity();
        inputs.angleMotorAngle = (angleMotorSim.getRotorPosition() * (Math.PI * 2)) % (Math.PI * 2);
        inputs.spinMotorAppliedCurrent = powerMotorSim.getAppliedVoltage();
        inputs.spinMotorPower = powerMotorSim.getAppliedVoltage() / 12;

    }

    @Override
    public void setSpinMotorPower(double power) {
        powerMotorSim.setControl(new DutyCycleOut(power));
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        angleMotorSim.setControl(new PositionVoltage(Units.radiansToRotations(angle)));
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotorSim.setControl(new DutyCycleOut(power));
    }


    @Override
    public void resetEncoder(double angle) {

    }
}
