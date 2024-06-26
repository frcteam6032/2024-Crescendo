/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;
  private double tv, tx, ta, ty;
  private int tid;
  private double distance;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;
  private final NetworkTableEntry m_led_entry;
  private boolean aligned = false;

  /**
   * Creates a new Vision.
   */
  public VisionSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_targetList = new ArrayList<Double>(MAX_ENTRIES);
    m_led_entry = m_limelightTable.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tv = m_limelightTable.getEntry("tv").getDouble(0);
    tx = m_limelightTable.getEntry("tx").getDouble(0);
    ta = m_limelightTable.getEntry("ta").getDouble(0);
    ty = m_limelightTable.getEntry("ty").getDouble(0);
    distance = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace")
        .getDoubleArray(new double[6])[2];

    tid = (int) m_limelightTable.getEntry("tid").getInteger(0);

    if (m_targetList.size() >= MAX_ENTRIES) {
      m_targetList.remove(0);
    }
    m_targetList.add(ta);
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    double sum = 0;

    for (Double num : m_targetList) {
      sum += num.doubleValue();
    }
    return sum / m_targetList.size();
  }

  public boolean isTargetValid() {
    return (tv == 1.0);
  }

  public void setLlLedMode(int mode) {
    m_led_entry.setDouble((mode));
  }

  public boolean isAligned() {
    // Making sure we actually have a target
    if (isTargetValid() == true) {
      if (tx < 3.0 && tx > -3.0) {
        aligned = true;
      } else {
        aligned = false;
      }
    }
    return aligned;
  }

  public int getTargetID() {
    return tid;
  }

  public double getTargetDistance() {
    /*
     * double targetOffsetAngle_Vertical = ty;
     * 
     * // Degrees from being vertical
     * double limelightMountAngleDegrees = 25.0;
     * 
     * // Distance from the floor to the limelight
     * double limelightLensHeightInches = 20.0;
     * 
     * // distance from the target to the floor
     * double goalHeightInches = 60.0;
     * 
     * double angleToGoalDegrees = limelightMountAngleDegrees +
     * targetOffsetAngle_Vertical;
     * double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
     * 
     * //calculate distance
     * double distanceFromLimelightToGoalInches = (goalHeightInches -
     * limelightLensHeightInches) / Math.tan(angleToGoalRadians);
     * 
     * return distanceFromLimelightToGoalInches;
     */
    return distance;
  }

}
