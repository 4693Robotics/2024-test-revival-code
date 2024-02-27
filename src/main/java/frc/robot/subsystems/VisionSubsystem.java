package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    //Creates new photon camera
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    PhotonPipelineResult result;
    List < PhotonTrackedTarget > tags;
    PhotonTrackedTarget bestTag;

    boolean isTagDetected;

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget AprilTagDetectedWidget = TeleopTab
        .add("Tag Detected", isTagDetected);

    /**
     * This subsystem contains the vision system for apriltag vision for the
     * 2024 CRESENDO FRC competition
     */
    public VisionSubsystem() {}

    @Override
    public void periodic() {
        
        //gets latest result from the camera
        result = camera.getLatestResult();

        //if statements for seting values to the targets and so they don't break all code with null values
        if (result.hasTargets()) {
            tags = result.getTargets();
            bestTag = result.getBestTarget();
        } else {
            tags = null;
            bestTag = null;
        }

        //Shuffleboard stuff
        SmartDashboard.putNumber("April tag x", this.getBestTagXDistance());
        SmartDashboard.putNumber("April tag y", this.getBestTagYDistance());
        SmartDashboard.putNumber("new april tag yawz", this.getBestTagYaw());

        //boolean for apriltag detection
        this.isTagDetected = this.isTagDetected();
    }

    /**
     * Returns the value of the best tags X distance
     * @return best tag X distance
     */
    public double getBestTagXDistance() {
        if (bestTag != null)
            return bestTag.getAlternateCameraToTarget().getX();
        else
            return -1;
    }

    /**
     * Returns the value of the best tags Y distance
     * @return best tag Y distance
     */
    public double getBestTagYDistance() {
        if (bestTag != null)
            return bestTag.getAlternateCameraToTarget().getY();
        else
            return -1;
    }

    /**
     * Returns the value of the best tags yaw rotation
     * @return best tag yaw rotation
     */
    public double getBestTagYaw() {
        if (bestTag != null)
            return bestTag.getBestCameraToTarget().getRotation().getAngle();
        else {
            return -0;
        }
    }

    /**
     * Returns the value of the selected tag X distance
     * @param TagID
     * @return selected tag X distance
     */
    public double getTagXDistance(int TagID) {
        if (tags != null) {
        for (PhotonTrackedTarget target: tags) {
            if (target.getFiducialId() == TagID) {
                return target.getAlternateCameraToTarget().getX(); // Found the target with the desired fiducial ID
            }
        }
        return -1;
        } else {
            return -0;
        }
    }

    /**
     * Returns the value of the selected tag Y distance
     * @param TagID
     * @return selected tag Y distance
     */
    public double getTagYDistance(int TagID) {
        if (tags != null) {
        for (PhotonTrackedTarget target: tags) {
            if (target.getFiducialId() == TagID) {
                return target.getAlternateCameraToTarget().getY(); // Found the target with the desired fiducial ID
            }
        }
        return -1;
        } else {
            return -0;
        }
    }

    /**
     * Returns the value of the selected tag yaw rotation
     * @param TagID
     * @return selected tag yaw rotation
     */
    public double getTagYaw(int TagID) {
        if (tags != null) {
        for (PhotonTrackedTarget target: tags) {
            if (target.getFiducialId() == TagID) {
                return target.getBestCameraToTarget().getRotation().getAngle(); // Found the target with the desired fiducial ID
            }
        }
        return -0;
    } else {
        return -0;
    }
  } 

  /**
   * returns a boolean equal to if the tag is detected
   * @return true if tag detected
   */
  public boolean isTagDetected() {
    if (bestTag != null)
        return true;
    else {
        return false;
    }
}
}