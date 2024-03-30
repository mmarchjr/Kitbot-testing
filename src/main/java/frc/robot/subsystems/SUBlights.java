/*  */// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUBlights extends SubsystemBase {

  AddressableLED lightStrand;
  AddressableLEDBuffer lightBuffer;

  /** Creates a new SUBunderglow. */
  public SUBlights() {
    lightStrand = new AddressableLED(Constants.kstrandPort);
    lightBuffer = new AddressableLEDBuffer(Constants.kStrandLength);

    lightStrand.setLength(Constants.kStrandLength);
    lightStrand.start();
    
  }

  public void set_full_strand(int R,int G,int B){
    SmartDashboard.putNumber("R", R);
    SmartDashboard.putNumber("G", G);
    SmartDashboard.putNumber("B", B);
    for(int i = 0; i<Constants.kStrandLength; i++){
        lightBuffer.setRGB(i,R,G,B);
    }
    lightStrand.setData(lightBuffer);
  }

  public void set_full_strand(int[] RGB){
    set_full_strand(RGB[0],RGB[1],RGB[2]);
  }

  public void set_pos_RGB(int pos,int r,int g,int b){
    lightBuffer.setRGB(pos, r, g, b);
  }
  
  public void set_pos_RGB(int pos, int[] rgb){
    lightBuffer.setRGB(pos, rgb[0], rgb[1], rgb[2]);
  }

  public void update(){
    lightStrand.setData(lightBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}