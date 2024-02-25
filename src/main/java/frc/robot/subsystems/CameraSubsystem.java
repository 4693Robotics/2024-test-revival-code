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

public class CameraSubsystem extends SubsystemBase {

    /** Creates a new Vision. */
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    PhotonPipelineResult result;
    List < PhotonTrackedTarget > tags;
    PhotonTrackedTarget bestTag;

    ShuffleboardTab TeleopTab = Shuffleboard.getTab("Teleop");

    SimpleWidget AprilTagDetectedWidget = TeleopTab
        .add("Tag Detected", this.isTagDetected());

    public CameraSubsystem() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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
        SmartDashboard.putNumber("new april tag yawz", this.get1BestTagYaw());
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

    public double get1BestTagYaw() {
        if (bestTag != null)
            return bestTag.getBestCameraToTarget().getRotation().getAngle();
        else {
            return -0;
        }
    }

    public double getTagXDistance(int TagID) {
        for (PhotonTrackedTarget target: tags) {
            if (target.getFiducialId() == TagID) {
                return target.getAlternateCameraToTarget().getX(); // Found the target with the desired fiducial ID
            }
        }
        return -1;
    }

    public double getTagYDistance(int TagID) {
        for (PhotonTrackedTarget target: tags) {
            if (target.getFiducialId() == TagID) {
                return target.getAlternateCameraToTarget().getY(); // Found the target with the desired fiducial ID
            }
        }
        return -1;
    }

    public double getTagYawDistance(int TagID) {
        for (PhotonTrackedTarget target: tags) {
            if (target.getFiducialId() == TagID) {
                return target.getBestCameraToTarget().getRotation().getAngle(); // Found the target with the desired fiducial ID
            }
        }
        return -1;
    }
}