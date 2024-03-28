package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.mathHelpers.Trigonometry;
public class ComputerAlign extends Command {
    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;

    public ComputerAlign(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;

        addRequirements(drivetrainSubsystem);
    }


    // AKA our C.A.S.S (computer assisted semi-alignment system)

    public boolean alignSide() {
    if (m_visionSubsystem.isTargetValid() == true) {
        // If were to the left, drive right
        if (m_visionSubsystem.getTX() < -3){
            m_drivetrainSubsystem.drive(0.0, 0.1, 0.0, false, false);
        }
        // If we're to the right, drive left
        else if (m_visionSubsystem.getTX() > 3) {
            m_drivetrainSubsystem.drive(0.0, -0.1, 0.0, false, false);
        }
        else {
             m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
             return true;
        }
    
    }
    else {
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
    return false;
}


public boolean alignDistance(int aprilTagID) {
   // This will change depending on the april tag

   // Amp ids = 6, 5
   // Speaker ids = 7, 4

   double distance = m_visionSubsystem.getTargetDistance();
   double distanceThreshold = 3;
   if (m_visionSubsystem.isTargetValid() == false) {
    return false;
   } 
   else {
    if (aprilTagID == 6 || aprilTagID == 5) {
        distanceThreshold = 1;
        if (distance > distanceThreshold) {
            m_drivetrainSubsystem.drive(0.1, 0.0, 0.0, false, false);
        } else {
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
            return true;
        }
    } else if (aprilTagID == 7 || aprilTagID == 4) {
        distanceThreshold = 10;
        if (distance > distanceThreshold) {
            m_drivetrainSubsystem.drive(0.1, 0.0, 0.0, false, false);
        } else {
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
            return true;
        }
    }
    else {
        if (distance > distanceThreshold) {
            m_drivetrainSubsystem.drive(0.1, 0.0, 0.0, false, false);
        } else {
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
            return true;
        }
    }
}
   return true;
}


public boolean alignYaw(double currentYaw) {
    // Get the position of the target in Cartesian coordinates
    double targetX = m_visionSubsystem.getTX();
    double targetY = m_visionSubsystem.getTY();
    double targetZ = m_visionSubsystem.getTargetDistance(); // Get distance
    
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
    
    // Set the speed for rotation based on the angle difference
    // Signnum tells the robot to go left or right
    double rotationSpeed = Math.signum(angleDifference) * 0.5; // Constant rotation speed

    double kAngleThreshold = 3;
    
    // Drive the robot based on the angle difference
    if (Math.abs(angleDifference) > kAngleThreshold) { // Adjust angle threshold
        // Rotate the robot towards the target angle
        m_drivetrainSubsystem.drive(0.0, 0.0, rotationSpeed, false, false);
    } else {
        // Stop the robot when aligned with the target angle
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
        return true;
    }
    
    // Return false if not aligned
    return false;
}




    @Override
    public void execute() {
     // Align left and right first
        if (alignSide() == true) {
            // Align distance
            if (alignDistance(m_visionSubsystem.getTargetID()) == true) {
                // Align yaw
                if (alignYaw(m_drivetrainSubsystem.getHeading()) == true) {
                    return;
                }
            }
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
