package frc.robot.utilities;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.interfaces.SwerveExtensionInterface;


public class ShooterUtilities {

    private Translation2d deltaTranslation;
    private ChassisSpeeds rotationSpeed;
    private double tolerance;
    private double rotationKP;
    private boolean atTolerance;
    private ChassisSpeeds robotSpeed = new ChassisSpeeds(); // Field relative robot speeds. Convert before passing.

    double velocityCompensationFactor;
    double testTolerance = 5; //  5 degrees tolerance


    
    private static final PIDController getRotationController() {
        PIDController controller = new PIDController(2.0, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
    /**
     * Construct a new TurnToPose object to determine the rotational speed to look at a certain position
     * @param marginPid The margin in degrees within which the desired speed is 0
     * @param kP The constant with which the difference in angle is multiplied to determine the speed
     */

    public double calculateRateToAlign(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
            double controllerVelX = -controller.getLeftY();
            double controllerVelY = -controller.getLeftX();

            Pose2d drivePose = new Pose2d();
            Pose2d targetPose = targetPoseSupplier.get();
            double shooterOffset = 0; // in meters, offset the pose of the shooter or whatever to calculate the angle.
            double targetDistance = drivePose.getTranslation().getDistance(targetPose.getTranslation());
            double shooterAngleRads = Math.acos(shooterOffset / targetDistance); 
            Rotation2d shooterAngle = Rotation2d.fromRadians(shooterAngleRads);
            Rotation2d offsetAngle = Rotation2d.kCCW_90deg.minus(shooterAngle);
            Rotation2d desiredAngle = offsetAngle.plus(drivePose.relativeTo(targetPose).getTranslation().getAngle()).plus(Rotation2d.k180deg);
            desiredAngle = desiredAngle.plus(Rotation2d.k180deg);
            Rotation2d currentAngle = drivePose.getRotation();
            Rotation2d deltaAngle = currentAngle.minus(desiredAngle);

            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);
            // if wrapped angle is less than a tolerance, we should exit the command if needed.
            
            double rotationalRate = getRotationController().calculate(currentAngle.getRadians(), desiredAngle.getRadians());
            return rotationalRate;
    }
    
    
    // Calculate the time of flight to a target height. Useful for leading shots at low velocities.
    public double calculateTOFToTarget(double shotVelocityMps, double shotAngleRad, double shooterHeightM, double targetHeightM) {
    double g = 9.81;
    double vy = shotVelocityMps * Math.sin(shotAngleRad);
    double deltaY = targetHeightM - shooterHeightM;

    double discriminant = vy * vy - 2 * g * deltaY;
    if (discriminant < 0) {
        // Target is unreachable
        return -1;
    }

    double t1 = (vy + Math.sqrt(discriminant)) / g;
    double t2 = (vy - Math.sqrt(discriminant)) / g;

    // Pick the positive, realistic time
    if (t1 > 0) return t1;
    if (t2 > 0) return t2;

    return -1; 

    }

}