package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {


    void setPipeLine(int pipeLineIndex);

    void updateInputs(VisionInputsLogged inputs);

    Result getLatestResult();

}
