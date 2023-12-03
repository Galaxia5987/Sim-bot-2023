package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class IntakeIOSim implements IntakeIO {
    //    private final FlywheelSim spinMotor;
//    private final SingleJointedArmSim angleMotor;
    private final TalonFXSimState angleMotorPhx;
    private final TalonFX talonFXAngle;
    private final TalonFX talonFXPower;
    private final TalonFXSimState powerMotorPhx;
    private final TalonFXConfigurator angleConfigurator;
    private final TalonFXConfigurator powerConfigurator;
//    private final PIDController angleFeedback;

    public IntakeIOSim() {
//        spinMotor = new FlywheelSim(DCMotor.getNEO(1), IntakeConstants.SPIN_GEAR_RATIO, 1);
//        angleMotor = new SingleJointedArmSim(LinearSystemId.createSingleJointedArmSystem(
//                DCMotor.getFalcon500(1), 2, IntakeConstants.ANGLE_GEAR_RATIO
//        ), DCMotor.getFalcon500(1), IntakeConstants.ANGLE_GEAR_RATIO, 0.7, -Math.PI * 2, Math.PI * 2, false, 0); //could be wrong type of motor or max/min angle
//        angleFeedback = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

        talonFXAngle = new TalonFX(0);
        talonFXPower = new TalonFX(1);


        angleMotorPhx = new TalonFXSimState(talonFXAngle);
        powerMotorPhx = new TalonFXSimState(talonFXPower);

        angleConfigurator = talonFXAngle.getConfigurator();
        powerConfigurator = talonFXPower.getConfigurator();


    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        inputs.spinMotorPower = powerMotorPhx.getMotorVoltage()/12;
        inputs.spinMotorCurrent = powerMotorPhx.getSupplyCurrent();
        inputs.angleMotorAngle = angleMotorPhx.setRotorVelocity()
        inputs.angleMotorcurrent = angleMotor.getCurrentDrawAmps();



        spinMotor.update(0.02);
        angleMotor.update(0.02);
    }

    @Override
    public void setSpinMotorPower(double power) {
        powerMotorPhx.setSupplyVoltage(power*12);
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        angleMotorPhx.setRawRotorPosition(angle/2*Math.PI);
    }


    @Override
    public void resetEncoder(double angle) {
return super;
    }
}
