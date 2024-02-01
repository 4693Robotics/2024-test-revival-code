package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    
    private PhotonCamera m_Photon1 = new PhotonCamera("photonvision");

    private PhotonTrackedTarget target;

    public CameraSubsystem() {
    }

    public void periodic() {
        PhotonPipelineResult photon1Result = m_Photon1.getLatestResult();

        target = photon1Result.getBestTarget();

        SmartDashboard.putNumber("Tag id", target.getFiducialId());
        SmartDashboard.putNumber("area", target.getArea());
        SmartDashboard.putNumber("Pitch", target.getPitch());
        SmartDashboard.putNumber("Skew", target.getSkew());
        SmartDashboard.putNumber("Yaw", target.getYaw());
        SmartDashboard.putNumber("Target x", target.getAlternateCameraToTarget().getX());
        SmartDashboard.putNumber("Target y", target.getAlternateCameraToTarget().getY());
        SmartDashboard.putNumber("Target z", target.getAlternateCameraToTarget().getZ());
    }  
    
    public PhotonTrackedTarget getTarget() {
        return target;
    }
}
