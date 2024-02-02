package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    
    private PhotonCamera m_Photon1 = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    private List<PhotonTrackedTarget> target;

    public CameraSubsystem() {
    }

    public void periodic() {
        PhotonPipelineResult photon1Result = m_Photon1.getLatestResult();

        target = photon1Result.getTargets();
    }  
    
    public List<PhotonTrackedTarget> getTarget() {
        return target;
    }
}
