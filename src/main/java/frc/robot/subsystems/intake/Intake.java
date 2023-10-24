package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.intake.commands.HoldIntakeInPlace;
import frc.robot.utils.units.UnitModel;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static Intake INSTANCE;
    private final CANSparkMax motor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(Ports.Intake.ANGLE_MOTOR);
    private final UnitModel unitModel = new UnitModel(IntakeConstants.TICKS_PER_DEGREE);
    private final IntakeLoggedInputs inputs = new IntakeLoggedInputs();
    private Command lastCommand = null;
    private boolean switchedToDefaultCommand = false;

    private Intake() {
        angleMotor.configFactoryDefault();
        motor.restoreFactoryDefaults();

        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        motor.setInverted(Ports.Intake.POWER_INVERTED);
        for (int i = 1; i <= 6; i++) {
            motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.fromId(i), 50000);
        }
        motor.burnFlash();

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        angleMotor.setInverted(Ports.Intake.ANGLE_INVERTED);
        angleMotor.config_kP(0, IntakeConstants.kP);
        angleMotor.config_kI(0, IntakeConstants.kI);
        angleMotor.config_kD(0, IntakeConstants.kD);
        angleMotor.config_kF(0, IntakeConstants.kF);
        angleMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                true, 30, 0, 0
        ));
        angleMotor.configClosedLoopPeakOutput(0, 0.4);
    }

    /**
     * @return the INSTANCE of the Intake.
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    /**
     * @return the relative output.
     * Return the power that the motor applies. [%]
     */
    private double getPower() {
        return motor.get();
    }

    /**
     * Set the motors' relative output.
     *
     * @param power is the power that the motor applies. [%]
     */
    public void setPower(double power) {
        inputs.setpointPower = power;
    }

    private double getAngleMotorVelocity() {
        return unitModel.toVelocity(inputs.velocity);
    }

    public void setAnglePower(double power) {
        inputs.setpointPower = power;
    }

    /**
     * @return the motor's position. [degrees]
     */

    public double getAngle() {
        return unitModel.toUnits(inputs.angle);
    }

    /**
     * Sets the angles position.
     *
     * @param angle is the angle of the retractor. [degrees]
     */
    public void setAngle(double angle) {
        inputs.angle = angle;
    }

    public Command lowerIntake() {
        return new InstantCommand(() -> resetEncoder(IntakeConstants.ANGLE_UP), this)
                .andThen(new RunCommand(() -> setAngle(-40), this));
    }

    public void resetEncoder(double angle) {
        angleMotor.setSelectedSensorPosition(
                unitModel.toTicks(angle)
        );
    }

    public double getCurrent() {
        return inputs.current;
    }

    public Command run(double power) {
        return new RunCommand(() -> this.setPower(power));
    }

    @Override
    public void periodic() {
        inputs.power = getPower();
        inputs.angle = getAngle();
        inputs.velocity = getAngleMotorVelocity();
        inputs.current = angleMotor.getSupplyCurrent();
        inputs.anglePower = angleMotor.getMotorOutputPercent();

        motor.set(inputs.power);
        angleMotor.set(TalonFXControlMode.PercentOutput, inputs.anglePower);
        angleMotor.set(ControlMode.Position, unitModel.toTicks(inputs.angle));

        var currentCommand = getCurrentCommand();
        switchedToDefaultCommand = (currentCommand instanceof HoldIntakeInPlace) &&
                !(lastCommand instanceof HoldIntakeInPlace);
        lastCommand = currentCommand;

        Logger.getInstance().processInputs("Intake", inputs);
    }

    public boolean switchedToDefaultCommand() {
        return switchedToDefaultCommand;
    }
}
