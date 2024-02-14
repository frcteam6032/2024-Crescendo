package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.mathHelpers.Trigonometry;
public class ComputerAlign extends Command {
    private final VisionSubsystem m_visionSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;
    private Trigonometry angleToTarget;

    public ComputerAlign(DriveSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    //TODO make sure we add the gyro to this command
    @Override
    public void execute() {
        angleToTarget = new Trigonometry();
        angleToTarget.calcuateAngle(m_visionSubsystem.getTX(), m_visionSubsystem.getTY());
        // If we're to the left
        if (m_visionSubsystem.isTargetValid() == true) {
        // If were to the left, drive right
        if (m_visionSubsystem.getTX() < 0)  {
            m_drivetrainSubsystem.drive(0.8, 0.0, 0.0, false, false);
        }
        // If we're to the right, drive left
        else if (m_visionSubsystem.getTX() > 0) {
            m_drivetrainSubsystem.drive(-0.8, 0.0, 0.0, false, false);
        }
    
    }
    else {
        m_drivetrainSubsystem.drive(0.0, 0.0, 0.0, false, false);
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
