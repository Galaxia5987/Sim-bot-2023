package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final VisionModule[] modules;
    private final Result[] results;

    private Vision(VisionModule... modules) {
        this.modules = modules;
        results = new Result[modules.length];
    }

    public static Vision getInstance() {
        if (INSTANCE == null) {
            if (Robot.isReal()) {
                INSTANCE = new Vision();
            } else {
                INSTANCE = new Vision();
            }
        }
        return INSTANCE;
    }

    public Result[] getResults() {
        return results;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            module.io.updateInputs(module.inputs);
            Logger.getInstance().processInputs("Vision/" + module.name, module.inputs);
            results[i] = module.io.getLatestResult();
        }
    }
}
