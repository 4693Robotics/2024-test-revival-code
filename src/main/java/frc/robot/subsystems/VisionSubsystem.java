package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    /** Creates a new Vision. */
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    PhotonPipelineResult result;
    List < PhotonTrackedTarget > tags;
    PhotonTrackedTarget bestTag;

    boolean isTagDetected;

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget AprilTagDetectedWidget = TeleopTab
        .add("Tag Detected", isTagDetected);

    public VisionSubsystem() {}

    @Override
    public void periodic() {
        
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            tags = result.getTargets();
            bestTag = result.getBestTarget();
        } else {
            tags = null;
            bestTag = null;
        }
        SmartDashboard.putNumber("April tag x", this.getBestTagXDistance());
        SmartDashboard.putNumber("April tag y", this.getBestTagYDistance());
        SmartDashboard.putNumber("new april tag yawz", this.getBestTagYaw());

        this.isTagDetected = this.isTagDetected();
    }
    public double getBestTagXDistance() {
        if (bestTag != null)
            return bestTag.getAlternateCameraToTarget().getX();
        else
            return -1;
    }

    public double getBestTagYDistance() {
        if (bestTag != null)
            return bestTag.getAlternateCameraToTarget().getY();
        else
            return -1;
    }

    public boolean isTagDetected() {
        if (bestTag != null)
            return true;
        else {
            return false;
        }
    }

    public double getBestTagYaw() {
        if (bestTag != null)
            return bestTag.getBestCameraToTarget().getRotation().getAngle();
        else {
            return -0;
        }
    }

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
}