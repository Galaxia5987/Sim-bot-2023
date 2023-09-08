package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision extends SubsystemBase {
    private final VisionInputsAutoLogged[] visionInputs;
    private final VisionIO[] io_s;
    private final Pose3d[] estimatedPoses;
    private static Vision INSTANCE;

    private Vision(VisionIO... io_s) {
        this.visionInputs = new VisionInputsAutoLogged[io_s.length];
        for (int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputsAutoLogged();
        }
        this.io_s = io_s;
        this.estimatedPoses = new Pose3d[this.visionInputs.length];

    }

    public static Vision getINSTANCE(){
        if(INSTANCE == null) {
            if (Robot.isReal()) {
                INSTANCE = new Vision(
                        new IOPhotonVision(new PhotonCamera("camera_1"), 0),
                        new IOPhotonVision(new PhotonCamera("camera_2"), 1)
                );

            }
            else{
                INSTANCE = new Vision(
                        new IOCameraSim(0 ,"simCam_1"),
                        new IOCameraSim(1 , "simCam_2")
                );
            }
        }
        return INSTANCE;
    }


    public void setPipeLine(int... pipeLine) {
        for (int i = 0; i < this.io_s.length; i++) {
            io_s[i].setPipeLine(pipeLine[i]);
        }
    }

    public Pose3d[] getEstimatedPoses(){
        return estimatedPoses;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < this.visionInputs.length; i++) {
            this.io_s[i].updateInputs(this.visionInputs[i]);
            Logger.getInstance().processInputs("Vision_" + (i+1), visionInputs[i]);

        }
        for(int i = 0; i < this.io_s.length; i++){
            this.estimatedPoses[i] = new Pose3d(new Translation3d(visionInputs[i].poseFieldOriented[0], visionInputs[i].poseFieldOriented[1], visionInputs[i].poseFieldOriented[2]),
                    new Rotation3d(new Quaternion(visionInputs[i].poseFieldOriented[3], visionInputs[i].poseFieldOriented[4], visionInputs[i].poseFieldOriented[5],visionInputs[i].poseFieldOriented[6])));
        }
    }
}
