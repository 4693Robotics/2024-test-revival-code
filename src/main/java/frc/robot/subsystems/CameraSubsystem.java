package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    
  /** Creates a new Vision. */
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  PhotonPipelineResult result;
  List<PhotonTrackedTarget> tags;
  PhotonTrackedTarget bestTag;
  public CameraSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    if(result.hasTargets()){
      tags = result.getTargets();
      bestTag = result.getBestTarget();
    }
    else{
      tags = null;
      bestTag = null;
    }

        SmartDashboard.putNumber("April Tag Yaw", this.getBestTagYaw());
  }
  public double getBestTagXDistance(){
    if(bestTag != null)
        return bestTag.getAlternateCameraToTarget().getX();
    else
      return -1;
  }

  public double getBestTagYDistance(){
    if(bestTag != null)
        return bestTag.getAlternateCameraToTarget().getY();
    else
      return -1;
  }

  public double getBestTagYaw(){
    if( bestTag != null)
      return bestTag.getYaw();
    else{
      return 4;
     }

 }
}
