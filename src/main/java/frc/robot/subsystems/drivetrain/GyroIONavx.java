package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utils.math.differential.Derivative;

public class GyroIONavx implements GyroIO {

    private final AHRS navx;

    private final Derivative yawDerivative = new Derivative(0, 0);
    private final Derivative pitchDerivative = new Derivative(0, 0);
    private final Derivative rollDerivative = new Derivative(0, 0);

    private double yawOffset = 0;

    public GyroIONavx() {
        navx = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();

        inputs.rawYawPositionRad = navx.getYaw();
        inputs.rawPitchPositionRad = navx.getPitch();
        inputs.rawRollPositionRad = navx.getRoll();

        yawDerivative.update(inputs.rawYawPositionRad);
        pitchDerivative.update(inputs.rawPitchPositionRad);
        rollDerivative.update(inputs.rawRollPositionRad);

        inputs.yawVelocityRadPerSec = yawDerivative.get();
        inputs.pitchVelocityRadPerSec = pitchDerivative.get();
        inputs.rollVelocityRadPerSec = rollDerivative.get();

        inputs.gyroVelocityX = navx.getVelocityX();
        inputs.gyroVelocityY = navx.getVelocityY();
    }
}
