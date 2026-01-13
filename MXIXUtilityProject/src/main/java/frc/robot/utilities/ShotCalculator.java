package frc.robot.utilities;

import frc.robot.utilities.ChassisAccelerations;
import frc.robot.fuelcharacterization.ShootOnTheFlyCalculator; 
 
import frc.robot.fuelcharacterization.ShootOnTheFlyCalculator.InterceptSolution;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



// stores current target and actively computes effective target

public class ShotCalculator extends SubsystemBase {
    public static final Transform3d BALL_TRANSFORM_CENTER = new Transform3d(-0.24, 0, 0.5, Rotation3d.kZero);

    // when implementing, include driveterain instance here.

    @Logged
    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    @Logged
    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    @Logged
    private Pose3d targetLocation = new Pose3d(); // Should be Hub pose.

    @Logged
    private double targetDistance = 0.0;

    @Logged

    private double targetSpeedRps = 8;

    public ShotCalculator() {
        // constructor should take a parameter of drivetrain.
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = new Pose2d();

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        targetSpeedRps = 0;  // DISTANCE_TO_SHOT_SPEED.get(targetDistance); Add interpolation table later.

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(BALL_TRANSFORM_CENTER);

        // both drivetrainSpeeds and accelerations need to be updated continuously.
        
        ChassisSpeeds drivetrainSpeeds = new ChassisSpeeds(); // drivetrain.getFieldRelativeSpeeds();
        ChassisAccelerations drivetrainAccelerations = ChassisAccelerations.getFieldRelativeAccelerations(drivetrainSpeeds); // implement properly later.

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose, targetLocation,
                drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
                5, 0.01);

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
    }

    public Pose3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }
}