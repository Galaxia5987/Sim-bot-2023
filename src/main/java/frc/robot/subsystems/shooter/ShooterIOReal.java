package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.utils.Utils;
import frc.robot.utils.motors.PIDTalon;

import frc.robot.utils.units.UnitModel;

import static frc.robot.Constants.TALON_TIMEOUT;

public class ShooterIOReal extends SubsystemBase implements ShooterIO{

    public static ShooterIOReal INSTANCE;

    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Shooter.MAIN_MOTOR);

    private final UnitModel unitModel = new UnitModel(ShooterConstants.TICKS_PER_ROTATION);

    private double setpoint = 0;

    public ShooterIOReal() {
        motor.configFactoryDefault(TALON_TIMEOUT);
        motor.setInverted(TalonFXInvertType.Clockwise);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.config_kP(0, ShooterConstants.PID_CONSTANTS.kP);
        motor.config_kI(0, ShooterConstants.PID_CONSTANTS.kI);
        motor.config_kD(0, ShooterConstants.PID_CONSTANTS.kD);
        motor.config_kF(0, ShooterConstants.Kf);
    }

        public static ShooterIOReal getInstance() {
            if (INSTANCE == null) {
                INSTANCE = new ShooterIOReal();
            }
            return INSTANCE;
        }

    @Override
    public void updateInput(ShooterInputs inputs) {}

    @Override
    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(velocity / 60));
        setpoint = velocity;
    }

    @Override
    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity()) * 60;
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public boolean atSetPoint(double tolerance) {
        return Utils.deadband(getVelocity() - setpoint, tolerance) == 0;
    }

}
