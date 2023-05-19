package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.units.UnitModel;

public class ShooterIOReal extends SubsystemBase {

    public static ShooterIOReal INSTANCE;

    private final WPI_TalonFX mainMotor = new WPI_TalonFX(0);
    private final WPI_TalonFX auxMotor = new WPI_TalonFX(0);

    private final UnitModel unitModel = new UnitModel(0);


    public ShooterIOReal(){
        PIDController pidController = new PIDController(0,0,0);
    }

    public static ShooterIOReal getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterIOReal();
        }
        return INSTANCE;
    }

    private void setVelocity(){

    }

    private int getVelocity(){
        unitModel.toVelocity(mainMotor.getSelectedSensorVelocity());
    }

}
