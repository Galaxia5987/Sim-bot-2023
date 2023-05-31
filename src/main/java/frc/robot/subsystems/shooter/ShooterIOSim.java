package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterIOSim extends SubsystemBase implements ShooterIO {
    public static ShooterIOSim INSTANCE;
    private final FlywheelSim simMotor;
    private final PIDController velocityFeedback;
    private double currentVelocity = 0;


    public ShooterIOSim(){
        simMotor = new FlywheelSim(
                DCMotor.getFalcon500(1), 1, ShooterConstants.MOMENT_OF_INERTIA);

        velocityFeedback = new PIDController(1, 0, 0);
    }

    public static ShooterIOSim getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterIOSim();
        }
        return INSTANCE;
    }

    @Override
    public void updateInput(ShooterInputs inputs){
        simMotor.update(0.02);
        inputs.velocity = simMotor.getAngularVelocityRPM();
        currentVelocity = simMotor.getAngularVelocityRPM();
    }

    @Override
    public void setVelocity(double velocity){
        currentVelocity = velocityFeedback.calculate(currentVelocity, velocity);
        simMotor.setInput(currentVelocity);
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public double getSetpoint() {
        return 0;
    }

}
