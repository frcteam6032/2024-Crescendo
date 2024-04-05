package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.Globals.GlobalVars;
import frc.robot.subsystems.DriveSubsystem;

public class autoDrive extends Command {
    private long startingTime;
    private final DriveSubsystem m_drivetrainSubsystem;



    public autoDrive(DriveSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;


        addRequirements(drivetrainSubsystem);
    }





    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();

    }


    @Override
    public void execute() {
        // 1 = middle
        // 2 = left 
        // 3 = right

        final double autoSpeed = 0.5;
        final double speakerAngle = 60.0 * (Math.PI / 180);
        
        final double x_Speed = Math.cos(speakerAngle)*autoSpeed;
        final double y_Speed = Math.sin(speakerAngle)*autoSpeed;

        if (Globals.GlobalVars.middleSpeaker == true) {
            m_drivetrainSubsystem.drive(-1*autoSpeed, 0, 0, false, false);
        }
        else if (Globals.GlobalVars.leftSpeaker == true) {
            m_drivetrainSubsystem.drive(-1*x_Speed, -1*y_Speed, 0, false, false);
        }
        else if (Globals.GlobalVars.rightSpeaker == true) {
            m_drivetrainSubsystem.drive(-1*x_Speed, y_Speed, 0, false, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(0, 0, 0, false, false);
    }




    private final int MaxTime = 3000;

    @Override
    public boolean isFinished() {
        long elapsedTime = System.currentTimeMillis() - startingTime;
        return elapsedTime > MaxTime ? true : false;
    }

}