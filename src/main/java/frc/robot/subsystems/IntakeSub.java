// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorMatch;

public class IntakeSub extends SubsystemBase {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 mySensor = new ColorSensorV3(i2cPort);
  boolean nodeIn = false;
  ColorMatch colorMatcher = new ColorMatch();
  Color NodeColor = new Color(0.92, 0.203, 0.91);
  Color detectedColor;
  double distance;

  PWMSparkMax blinkinPWM = new PWMSparkMax(9);

  CANSparkMax leftSide;
  CANSparkMax rightSide;
  CANSparkMax intakeRight;
  CANSparkMax intakeLeft;
  RelativeEncoder myEncoder = rightSide.getEncoder();
  double myAngle;

  /** Creates a new colorSensor. */
  public IntakeSub() {
    colorMatcher.addColorMatch(NodeColor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectNode();
    SmartDashboard.putBoolean("node in: ", nodeIn);
    SmartDashboard.putNumber("Detected red: ", mySensor.getRed());
    SmartDashboard.putNumber("Detected green: ", mySensor.getGreen());
    SmartDashboard.putNumber("Detected blue: ", mySensor.getBlue());
    SmartDashboard.putNumber("distance: ", distance);
    nodeIndicator(nodeIn);

    
    leftSide = new CANSparkMax(0, MotorType.kBrushless);
    rightSide = new CANSparkMax(1, MotorType.kBrushless);
    intakeLeft = new CANSparkMax(2, MotorType.kBrushless);
    intakeRight = new CANSparkMax(3, MotorType.kBrushless);

    SmartDashboard.putNumber("angle", myAngle);    
  }

  public void detectNode(){
    detectedColor = mySensor.getColor();
    
    distance = mySensor.getProximity();
   
    if(mySensor.getRed() >= mySensor.getBlue() && mySensor.getRed()>=mySensor.getGreen()||mySensor.getGreen()-mySensor.getRed()<=10&&mySensor.getProximity()>50){
      nodeIn = true;
    }else{
      nodeIn = false;
    }
    
  }

  public void nodeIndicator(boolean nodeStatus){
    if(nodeStatus){
      blinkinPWM.set(0.77); // green
    }else{
      blinkinPWM.set(0.61); // red
    }
  }

  public void extendIntake(){
    myAngle = (myEncoder.getPosition() / 66)*360;

    if(myAngle <= 70){
    leftSide.set(0.25);
    //rightSide.set(0.25);
    }
    
  }

  public void takeNode(){
    leftSide.set(0);
    //rightSide.set(0);

    intakeLeft.set(0.25);
    //intakeRight.set(0.25);

    if(nodeIn){
      intakeLeft.set(0);
      //intakeRight.set(0);
    }
  }

  public void retractIntake(){
    leftSide.set(-0.25);
    //rightSide.set(-1);
  }

}