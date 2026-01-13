package frc.robot.interfaces;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Interface for systems that can convert a difference in {@code Pose2d} objects to a {@code ChassisSpeeds}
 */
public interface SwerveExtensionInterface {

    /**
     * Converts a target and current {@code Pose2d} into an appropriate {@code ChassisSpeeds}
     *
     * @param currentPose the current pose of the robot
     * @param targetPose the desired position of the robot
     * @return the {@code ChassisSpeeds} to get from the current pose to the target pose
     */
    ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose);

    
    /**
     * Update the utility with a {@code Pose2d} of the current robot pose
     * @param currentChassisSpeeds the current pose of the robot
     */
    ChassisSpeeds updateRobotSpeeds(ChassisSpeeds currentChassisSpeeds);
}