package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.LoggedSubsystem;


public class ProximitySensor extends LoggedSubsystem<ProximitySensorLoggedInputs> {
    private static ProximitySensor INSTANCE;
    private final DigitalInput beam = new DigitalInput(9);

    private ProximitySensor() {
        super(new ProximitySensorLoggedInputs());
    }

    public static ProximitySensor getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ProximitySensor();
        }
        return INSTANCE;
    }

    /**
     * @return whether the beam breaker is blocked.
     */

    public boolean isBeamBlocked() {
        return !beam.get();
    }

    /**
     * Update the logger variables.
     */
    @Override
    public void updateInputs() {
        loggerInputs.ProximitySensorState = isBeamBlocked();
    }

    @Override
    public String getSubsystemName() {
        return "ProximitySensor";
    }
}
