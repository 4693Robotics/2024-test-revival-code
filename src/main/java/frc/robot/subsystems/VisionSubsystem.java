package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTag2024Constants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    //Creates new photon camera
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    PhotonPipelineResult result;
    List<PhotonTrackedTarget> tags;
    PhotonTrackedTarget bestTag;

    AprilTagFieldLayout aprilTagFieldLayout;

    PhotonPoseEstimator photonPoseEstimator;

    int SpeakercenterID;
    int SpeakerSideID;
    int AmpID;
    int SourceRightID;
    int SourceLeftID;

    Transform3d robotpose;

    boolean isTagDetected;

    ShuffleboardTab PreGameTab = Shuffleboard.getTab(ShuffleboardConstants.kPreGameTabName);
    ShuffleboardTab TeleopTab = Shuffleboard.getTab(ShuffleboardConstants.kTeleopTabName);

    SimpleWidget AprilTagDetectedWidget = TeleopTab
        .add("Tag Detected", isTagDetected);

    /*SimpleWidget SpeakerDistanceFromWallWidget = TeleopTab
    .add("Distance From Speaker Wall", getDistanceFromSpeakerWall())
    .withWidget(BuiltInWidgets.kTextView);*/

    /**
     * This subsystem contains the vision system for apriltag vision for the
     * 2024 CRESENDO FRC competition
     */
    public VisionSubsystem() {

        try {

            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );

            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              VisionConstants.kcameraPose);

        } catch (IOException e) {

            DriverStation.reportWarning(
                "April Tag layout could not be loaded", false);
        }

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            SpeakercenterID = AprilTag2024Constants.kRedSpeakerCenter;
            SpeakerSideID = AprilTag2024Constants.kRedSpeakerSide;
            AmpID = AprilTag2024Constants.kRedAmp;
            SourceRightID = AprilTag2024Constants.kRedSourceRight;
            SourceLeftID = AprilTag2024Constants.kRedSourceLeft;
        } else {
            SpeakercenterID = AprilTag2024Constants.kBlueSpeakerCenter;
            SpeakerSideID = AprilTag2024Constants.kBlueSpeakerSide;
            AmpID = AprilTag2024Constants.kBlueAmp;
            SourceRightID = AprilTag2024Constants.kBlueSourceRight;
            SourceLeftID = AprilTag2024Constants.kBlueSourceLeft;
        }
    }

    @Override
    public void periodic() {
        //boolean for apriltag detection
        this.isTagDetected = this.isTagDetected();
        AprilTagDetectedWidget.getEntry().setBoolean(isTagDetected);
        
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

        if (result.getMultiTagResult().estimatedPose.isPresent) {
            robotpose = result.getMultiTagResult().estimatedPose.best;
        }

        //Shuffleboard stuff
        //SpeakerDistanceFromWallWidget.getEntry().setDouble(getDistanceFromSpeakerWall());
    }

    public Optional<PhotonTrackedTarget> getTarget(int id) {
        if (tags != null) {
            for (PhotonTrackedTarget target : tags) {
                if (target.getFiducialId() == id) return Optional.of(target);
            }
        } else {
        return Optional.empty();
        }
        return Optional.empty();
    }

    public Optional<Pose3d> getTargetPose(PhotonTrackedTarget target) {
        int fiducialId = target.getFiducialId();
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);
        // Ensure the existence of this tag id, print warning
        if (tagPose.isEmpty()) {
            DriverStation.reportWarning("Fiducial id " + fiducialId + " not recognized", false);
        }
        // Return with optional null value
        return tagPose;
    }
    
    public double getDistanceToTarget(PhotonTrackedTarget target) {
        Optional<Pose3d> tagPose = getTargetPose(target);
        // Ensure the existence of this tag id
        if (tagPose != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            return Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
          
        } else {
            return -1;
        }
    }

    /**
     * returns the direct distance from the april tag
     * like the hypotonuse
     * @return distance in meters from speaker tag id
     */
    public double getDistanceFromSpeakerTag() {
        
        Optional<PhotonTrackedTarget> target = getTarget(SpeakercenterID);

        if (target.isPresent()) {
            return getDistanceToTarget(target.get());
        } else {
            return -1;
        }

    }

    public double getDistanceFromSpeakerWall() {
        
        Optional<PhotonTrackedTarget> target = getTarget(SpeakercenterID);

        if (target != null) {
            double hypotonuse = getDistanceToTarget(target.get());
            double leg =  1.4511020 - VisionConstants.kCameraDistanceFromGround; 
            return Math.sqrt(Math.pow(hypotonuse, 2) - Math.pow(leg, 2));
        } else {
            return -1;
        }
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

  public boolean isTagDetected() {
    if (result != null) {
    return result.hasTargets();
    } else  {
      return false;
    }  
  }

  public Transform3d getRobotPose() {
    return robotpose;
  }
}