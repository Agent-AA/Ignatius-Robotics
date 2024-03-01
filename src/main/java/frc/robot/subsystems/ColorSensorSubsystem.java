// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Color Sensor Subsystem manage the color sensor mounted by in the note loading area.
 * It is used to determine whether a note is loaded or not.
 */
public class ColorSensorSubsystem extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private double IR = m_colorSensor.getIR(); // infrared value
  public Color detectedColor;

  public ColorSensorSubsystem() {}

  /**
     * The displayColors method will output the red, green, blue, IR, and proximity values
     * of the color sensor to the SmartDashboard. It also prints the detectedColor (hexadecimal)
     * to the dashboard.
     */
  public void displayColors() {

    detectedColor = m_colorSensor.getColor();
    int proximity = m_colorSensor.getProximity();


    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);


    System.out.println(detectedColor);
  }

    /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleCommand () {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
