package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.mathHelpers.Trigonometry;

public class ComputerAlign extends Command {
    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;

    private double velocity_Y = 0.0;
    private double velocity_X = 0.0;
    private double velocity_R = 0.0;

    public ComputerAlign(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    // AKA our C.A.S.S (computer assisted semi-alignment system)

    public void alignSide() {
        if (m_visionSubsystem.isTargetValid() == true) {
            // If were to the left, drive right
            if (m_visionSubsystem.getTX() < -3) {
                velocity_Y = 0.1;
            }
            // If we're to the right, drive left
            else if (m_visionSubsystem.getTX() > 3) {
                velocity_Y = -0.1;
            } else {
                velocity_Y = 0;
                return;
            }

        } else {
            velocity_Y = 0;
        }
        return;
    }

    public void alignDistance(int aprilTagID) {
        // This will change depending on the april tag

        // Amp ids = 6, 5
        // Speaker ids = 7, 4

        // Make positive
        // Distance is in meters
        double distance = m_visionSubsystem.getTargetDistance() * -1;
        double distanceThreshold = 1;
        if (m_visionSubsystem.isTargetValid() == false) {
            return;
        } else {
            if (aprilTagID == 6 || aprilTagID == 5) {
                distanceThreshold = 0.5;
                if (distance > distanceThreshold) {
                    velocity_X = 0.1;
                } else {
                    velocity_X = 0;
                    return;
                }
            } else if (aprilTagID == 7 || aprilTagID == 4) {
                distanceThreshold = 4;
                if (distance > distanceThreshold) {
                    velocity_X = 0.1;
                } else {
                    velocity_X = 0;
                    return;
                }
            } else {
                distanceThreshold = 0.5;
                if (distance > distanceThreshold) {
                    velocity_X = 0.1;
                } else {
                    velocity_X = 0;
                    return;
                }
            }
        }
        return;
    }

    public void alignYaw(double currentYaw) {
        // Get the position of the target in Cartesian coordinates
        double targetX = m_visionSubsystem.getTX();
        double targetY = m_visionSubsystem.getTY();
        double targetZ = m_visionSubsystem.getTargetDistance() * -1; // Get distance
        // Make positive

        // Convert Cartesian coordinates to spherical coordinates
        double[] sphericalCoordinates = Trigonometry.cartesianToSpherical(targetX, targetY, targetZ);
        double azimuth = sphericalCoordinates[0];

        // Calculate the angle to align with based on azimuth
        double angleToTarget = Math.toDegrees(azimuth);

        // Calculate the difference between current yaw and target angle
        double angleDifference = angleToTarget - currentYaw;

        // The difference is between -180 and 180 degrees
        if (angleDifference > 180) {
            angleDifference -= 360;
        } else if (angleDifference <= -180) {
            angleDifference += 360;
        }

        final double kAngleThreshold = 3;

        // Drive the robot based on the angle difference
        if (Math.abs(angleDifference) > kAngleThreshold) { // Adjust angle threshold
            // Rotate the robot towards the target angle
            // Set the speed for rotation based on the angle difference
            // Signnum tells the robot to go left or right
            velocity_R = Math.signum(angleDifference) * 0.2; // Constant rotation speed
        } else {
            // Stop the robot when aligned with the target angle
            velocity_R = 0;
            // AKA when theta is less than 3 degrees
            return;
        }

        // Return false if not aligned
        return;
    }

    @Override
    public void execute() {
        // Instead of having each method move the bot (which causes only one method to
        // run, we will have global velocity variables)
        if (m_visionSubsystem.isTargetValid() == true) {
            alignSide();
            alignDistance(m_visionSubsystem.getTargetID());
            alignYaw(m_drivetrainSubsystem.getHeading());
            m_drivetrainSubsystem.drive(velocity_X, velocity_Y, velocity_R, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
