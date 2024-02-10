// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <ctre/Phoenix.h>
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>


// GLOBALS

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &rightside_leader);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &leftside_leader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftside_leader.SetInverted(true);
    leftside_follower.Follow(leftside_leader);
    leftside_follower.SetInverted(true);
    rightside_follower.Follow(rightside_leader);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
    

    frc::CameraServer::StartAutomaticCapture();

  }

class armClass {
  public:
    int position;
    void move_arm(int increment){
      std::cout << increment << "\n";
    }
};

  void AutonomousInit() override { m_timer.Restart(); }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 1_s) {// Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.5, 0.0
      , false);
      //  std::cout << encoderDistance << "\n";
    } else {
      // Stop robot
      m_robotDrive.StopMotor();
    }
  }

  void TeleopInit() override {
    m_encoder.Reset();
  }

  void TeleopPeriodic() override {
    m_robotDrive.ArcadeDrive(m_controller.GetLeftX() * 0.5, m_controller.GetLeftY() * 0.5);
  }

  void TestInit() override {
    m_encoder.Reset();
  }

  void TestPeriodic() override {
  double encoderRate = m_encoder.GetRate();
  double distancePerPulse = m_encoder.GetDistancePerPulse();
  double encoderCount = m_encoder.Get();
  double encoderDistance = m_encoder.GetDistance();
  double ArcadeMoveA;
  double ArcadeMoveB;
  
   ArcadeMoveA = -m_controller.GetLeftY()*0.5;
   ArcadeMoveB = -m_controller.GetRightX()*0.5;

  m_robotDrive.ArcadeDrive(ArcadeMoveA, ArcadeMoveB);
  frc::SmartDashboard::PutNumber("Encoder distance: ", encoderDistance);
  
  //drive robot

  //------------------------------------------------------------------------------------------------------------

    //  if (m_controller.GetYButton()){ // if Y button is pressed
    //    m_robotDrive.ArcadeDrive(0.3, 0.3); //set speed to 0.4 for variable speed = maxspeed*(1 - encoderDistance/2910) #or max encoder distance)
    //    if (encoderDistance >= 970){
    //     m_robotDrive.ArcadeDrive(0.3*(1-encoderDistance/2910), 0.3*(1-encoderDistance/2910));}
  
    //}
    //  if (m_limitswitch.Get()) { //if limit switch is tripped OR encoder distance is >= 53
    //    m_robotDrive.ArcadeDrive(0.0, 0.0); //stop motor altogether
    //   } 

    // std::cout <<("Go!\n");
    // std::cout << "Encoder rate: " << encoderRate << "\n";
    //  std::cout << "Encoder Distance: " << encoderDistance << "\n";
    // std::cout << "Distance Per Pulse: " << distancePerPulse << "\n";
    // std::cout << "Encoder count: " << encoderCount << "\n";

  }

 private:
  // Robot drive system
  WPI_TalonSRX rightside_leader{1};
  WPI_VictorSPX rightside_follower{2};
  WPI_VictorSPX leftside_leader{3};
  WPI_TalonSRX leftside_follower{4};

  frc::DifferentialDrive m_robotDrive{
      [&](double output) { rightside_leader.Set(output); },
      [&](double output) { leftside_leader.Set(output); }};

  frc::XboxController m_controller{1};
  frc::Timer m_timer;
  frc::DigitalInput m_limitswitch{2};
  frc::Encoder m_encoder{0,1};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif