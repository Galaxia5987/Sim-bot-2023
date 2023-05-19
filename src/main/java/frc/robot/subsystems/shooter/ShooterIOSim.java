package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.utils.units.UnitModel;


public class ShooterIOSim {
    private final FlywheelSim simMotor;
    private PIDController pidController;


    public ShooterIOSim(){
        simMotor = new FlywheelSim(
                DCMotor.getFalcon500(2), 1, 0);

        pidController = new PIDController(0, 0, 0);
    }


    @Override
    public void updateInput(){
        simMotor.update(0.02);

    }

    private void setVelocity(){

    }

    private int getVelocity(){

    }


}
