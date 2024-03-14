
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ShooterSubsystem extends SubsystemBase {

private CANSparkMax motorController1;
private CANSparkMax motorController2;

    public ShooterSubsystem() {
 
    motorController1 = new CANSparkMax(11, MotorType.kBrushless);
    motorController1.setInverted(true);

    motorController2 = new CANSparkMax(12, MotorType.kBrushless);
    motorController2.setInverted(false); // Inverting right motor rotation to intake objects

    }

    public void set_speed(double value){
        motorController1.set(value);
        motorController2.set(value);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}