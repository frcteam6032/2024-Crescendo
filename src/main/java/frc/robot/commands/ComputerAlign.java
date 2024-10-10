package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.Logging.Logger;

public class ComputerAlign extends Command {

    Logger logger = new Logger();

    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;

    private double velocity_Y = 0.0;
    private double velocity_X = 0.0;
    private double velocity_R = 0.0;

    public ComputerAlign(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;

        addRequirements(drivetrainSubsystem, visionSubsystem);
    }

    // Computer Assisted Semi-Alignment System (C.A.S.S)

    public void alignSide() {
        double tx = m_visionSubsystem.getTX();
        double kP = 0.05; // Proportional constant for side alignment
        double error = tx;

        if (Math.abs(error) > 3) { // Threshold in degrees
            // Adjust velocity based on error
            velocity_Y = -kP * error;
            // Limit maximum speed
            velocity_Y = Math.max(Math.min(velocity_Y, 0.3), -0.3);
        } else {
            velocity_Y = 0;
        }
    }

    public void alignDistance(int aprilTagID) {
        /*
         * / // Get the target distance in meters
         * double distance = Math.abs(m_visionSubsystem.getTargetDistance());
         * 
         * if (distance <= 0 || !m_visionSubsystem.isTargetValid()) {
         * velocity_X = 0;
         * return;
         * }
         * 
         * // Set distanceThreshold based on aprilTagID
         * double distanceThreshold;
         * switch (aprilTagID) {
         * case 6:
         * distanceThreshold = 0.8;
         * break;
         * case 5:
         * distanceThreshold = 1.0;
         * break;
         * case 7:
         * case 4:
         * distanceThreshold = 2.0;
         * break;
         * default:
         * distanceThreshold = 0.8;
         * break;
         * }
         * 
         * double error = distance - distanceThreshold;
         * 
         * if (error > 0) {
         * double kP = 0.1; // Proportional constant for distance alignment
         * velocity_X = kP * error;
         * // Limit maximum speed
         * velocity_X = Math.min(velocity_X, 0.3);
         * } else {
         * velocity_X = 0;
         * }
         */
        double kP = 0.1;
        double distanceVelocity = kP * m_visionSubsystem.getTY();
        distanceVelocity *= Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        distanceVelocity *= -1.0;
        velocity_X = distanceVelocity;
    }

    public void alignYaw(double currentYaw) {
        /*
         * if (!m_visionSubsystem.isTargetValid()) {
         * velocity_R = 0;
         * return;
         * }
         * 
         * // Get the target distance
         * double targetY = Math.abs(m_visionSubsystem.getTargetDistance());
         * 
         * // Ensure angle is in degrees and convert to radians where necessary
         * double targetX = Trigonometry.distanceX(currentYaw, targetY);
         * 
         * double targetZ = 0;
         * 
         * // Convert Cartesian coordinates to spherical coordinates
         * double[] sphericalCoordinates = Trigonometry.cartesianToSpherical(targetX,
         * targetY, targetZ);
         * double azimuthRadians = sphericalCoordinates[0];
         * double angleToTarget = Math.toDegrees(azimuthRadians);
         * 
         * // Calculate the difference between current yaw and target angle
         * double angleDifference = angleToTarget - currentYaw;
         * 
         * // Normalize angleDifference to be within -180 to 180 degrees
         * angleDifference = ((angleDifference + 180) % 360) - 180;
         * 
         * final double kAngleThreshold = 1; // Degrees
         * 
         * if (Math.abs(angleDifference) > kAngleThreshold) {
         * double kP = 0.01; // Proportional constant for rotation
         * velocity_R = kP * angleDifference;
         * // Limit maximum rotation speed
         * velocity_R = Math.max(Math.min(velocity_R, 0.5), -0.5);
         * } else {
         * velocity_R = 0;
         * }
         */

        double kP = 0.035;
        double angleVelocity = kP * m_visionSubsystem.getTX();
        angleVelocity *= Constants.DriveConstants.kMaxAngularSpeed;
        angleVelocity *= -1.0;
        velocity_R = angleVelocity;
    }

    @Override
    public void execute() {
        if (m_visionSubsystem.isTargetValid()) {
            alignSide();
            alignDistance(m_visionSubsystem.getTargetID());
            alignYaw(m_drivetrainSubsystem.getHeading());
            // m_drivetrainSubsystem.drive(velocity_X, velocity_Y, velocity_R, false,
            // false);
        } else {
            // Stop the robot if the target is not valid
            velocity_R = 0;
            velocity_X = 0;
            velocity_Y = 0;
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
        logger.info("Correcting YAW with rotational velocity [%.1f] ",
                velocity_R);
        logger.info("Correcting side with velocity [%.1f]",
                velocity_Y);
        logger.info("Aligning Depth with velocity [%.1f]",
                velocity_X);

    }

    @Override
    public void end(boolean interrupted) {
        velocity_R = 0;
        velocity_X = 0;
        velocity_Y = 0;
        // Stop the robot when the command ends
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
