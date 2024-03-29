// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUBlights;
import edu.wpi.first.math.MathUtil;

public class CMDlights extends Command {
  Timer lightTimer = new Timer();

  SUBlights subGlow;

  private static SendableChooser<Boolean> ShooterLightChooser = new SendableChooser<Boolean>();
  Notifier loadingNotifier;

  /** Creates a new CMDUnderglow. */
  public CMDlights(SUBlights subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    subGlow = subsystem;
    addRequirements(subGlow);

    loadingNotifier = //Based on https://github.com/Mechanical-Advantage/RobotCode2023/blob/f0c26dc20e0f324a2093a4c81c687d41a120a07d/src/main/java/org/littletonrobotics/frc2023/subsystems/leds/Leds.java#L90-L103
        new Notifier(
            () -> {
              subGlow.set_full_strand((getBreatheColor(Constants.kblueRGB, Constants.kyellowRGB, System.currentTimeMillis()/1000)));
              //subGlow.set_full_strand(getBlinkColor(1.5, 1));
            }
        );
    loadingNotifier.startPeriodic(0.02);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lightTimer.reset();
    lightTimer.start();

    ShooterLightChooser.setDefaultOption("Light", true);
    ShooterLightChooser.addOption("no light", false);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isEnabled()) {
      subGlow.set_full_strand(0,0,0);
    lightTimer.stop();
    loadingNotifier.stop();
    }
    //loadingNotifier.stop(); //ensure the breath effect for early view stops when anything else is ready

    //int[] breatheRGB = getBreatheColor(Constants.kyellowRGB, Constants.kblueRGB, lightTimer.get());
    //subGlow.set_full_strand(breatheRGB[0], breatheRGB[1], breatheRGB[2]); //pass the color to the strand
    //sweep( (int) (lightTimer.get()*Constants.kStrandLength) );
    if(RobotContainer.kSUBShooter.getRPM() > 5000 && ShooterLightChooser.getSelected()){
      subGlow.set_full_strand(0,255,0);
    }
    subGlow.update();
  }

  int[] getBreatheColor(int[] yellow, int[] blue, double time){
    double controlValue = Math.sin(time*2); 
    double yellowness = MathUtil.clamp(controlValue,0,1);
    double blueness = MathUtil.clamp(-controlValue,0,1);
    //SmartDashboard.putNumber("blueness",blueness);
    //SmartDashboard.putNumber("yellowness", yellowness);

    int[] ret = {0,0,0};
    
    for(int i = 0; i<3; i++){
      ret[i] =(int) dualLerp(yellow[i],blue[i],yellowness,blueness); //cast the component value to an int so we can use it
    }

    return ret;
  }

  int[] getBlinkColor(double speed,double time){
    if(Math.signum(Math.sin(time*speed))>0){
      return Constants.kblueRGB;
    }
    else{
      return Constants.kyellowRGB;
    }
  }

  public void sweep(int position){ //not fully understood, result of trial and error
    int direction = 1;

    if((position/Constants.kStrandLength)%2==1){//if on even trip
      direction *= -1; //invert direction
    }

    position = position % Constants.kStrandLength; //so position is bounded

    if (direction == 1){ // sweep forward
      SmartDashboard.putString("sweepdir", "forward");
      for (var i = 0;i<Constants.kStrandLength;i++){
        if(i>position){
          subGlow.set_pos_RGB(i, Constants.kyellowRGB);
        }
        else {
          subGlow.set_pos_RGB(i,Constants.kblueRGB);
        }
      }
    }
    else{ //sweep back
      //SmartDashboard.putString("sweepdir", "back");
      for (var i = 0;i<Constants.kStrandLength;i++){
        if(i<Constants.kStrandLength - position){
          subGlow.set_pos_RGB(i, Constants.kblueRGB);
        }
        else {
          subGlow.set_pos_RGB(i,Constants.kyellowRGB);
        }
      }
    }
  }
  double dualLerp(double value1,double value2,double pos1,double pos2){//lerp for one component 
    return lerp(0,value1,pos1) + lerp(0,value2,pos2);//at least one lerp should always be zero
  }

  public static double lerp(double a, double b, double f) {//convenience function from http://www.java2s.com/example/java-utility-method/lerp/lerp-double-a-double-b-double-f-4ce8e.html
    return a + f * (b - a);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override 
  public boolean runsWhenDisabled() {
    return true;
  }
}