package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

@Logged
public class PhotonIO implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public PhotonIO(String cameraName, Transform3d cameraTransform, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraName);

        estimator = new PhotonPoseEstimator(fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform);
    }

    public void updateInputs(VisionInputs visionInputs) {
        // Assume pose will not be updated.
        visionInputs.poseUpdated = false;

        // Values used by the CLOSEST_TO_LAST_POSE and CLOSEST_TO_REFERENCE_POST strategies.
        estimator.setLastPose(visionInputs.estimatedPose);
        estimator.setReferencePose(visionInputs.estimatedPose);

        // Read all the new pipeline results since last time we checked, just return if there are none.
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty())
            return;

        // Generate a new estimated pose using the most recent pipeline result.
        Optional<EstimatedRobotPose> newPose = estimator.update(results.getLast());

        // If there is no new pose then we have nothing to do, just bail out.
        if (newPose.isEmpty()) {
            return;
        }

        EstimatedRobotPose newEstimate = newPose.get();
        visionInputs.estimatedPose = newEstimate.estimatedPose.toPose2d();
        visionInputs.timestampSeconds = newEstimate.timestampSeconds;

        // Get the list of targets used in the current estimate and store their Ids in our Inputs.
        List<PhotonTrackedTarget> targets = newEstimate.targetsUsed;
        int[] ids = new int[targets.size()];
        int index = 0;
        for (PhotonTrackedTarget target : targets) {
            ids[index++] = target.getFiducialId();
        }
        visionInputs.targetIds = ids;
        visionInputs.poseUpdated = true;
    }
}
