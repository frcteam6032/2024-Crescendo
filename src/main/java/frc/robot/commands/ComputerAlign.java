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

    //TODO make sure we add the gyro to this command

    // AKA our C.A.S.S (computer assisted semi-alignment system)

    public static double calculateYawAngle(double dx, double dy) {
         // Calculate angle in radians
         double angleRadians = Math.atan2(dy, dx);

         // Convert angle from radians to degrees
         double angleDegrees = Math.toDegrees(angleRadians);
 
         // Ensure angle is between 0 and 360 degrees
         angleDegrees = (angleDegrees + 360) % 360;
 
         return angleDegrees;
    }



    public boolean alignX() {
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


public boolean alignY() {
    if (m_visionSubsystem.isTargetValid() == true) {
      // If we are more than 3 units away from the target, drive forward
      if (m_visionSubsystem.getTY() > 3){
          m_drivetrainSubsystem.drive(0.1, 0.0, 0.0, false, false);
      }
      else {
        // If we are less than 3 units, stop
            m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
            return true;
      }
    
    }
    else {
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
    return false;
}


public boolean alignZ(double currentYaw) { 
    // We are working with degrees here
    // We need to get the amount of degrees that we are from the target
    // We will use the trigonometry class to get the angle we need to turn
    // We will then use the gyro to turn the robot to that angle
    double angleToTarget = calculateYawAngle(m_visionSubsystem.getTX(), m_visionSubsystem.getTY());
    if (m_visionSubsystem.isTargetValid() == true) {
    if (currentYaw < angleToTarget) {
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.1, false, false);
    }
    else if (currentYaw > angleToTarget) {
        m_drivetrainSubsystem.drive(0.0, 0.0, -0.1, false, false);
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



    @Override
    public void execute() {
        if (alignX() == true) {
            if (alignY() == true) {
                alignZ(m_drivetrainSubsystem.getHeading());
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
