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


  void AutonomousInit() override { m_timer.Restart(); }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 2_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(1, 0.0, false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
  }

  void TestInit() override {}

  void testFunction() {
       //Drive with arcade style (use right stick to steer)
       if (ourLimitSwitch.Get()) {
        m_robotDrive.ArcadeDrive(0.0, 0.0);
       } else {
         m_robotDrive.ArcadeDrive(-m_controller.GetAButton(),
                              m_controller.GetBButton());
       }
  }

  void TestPeriodic() override {
    testFunction();

  if (m_controller.GetYButton()){
    leftside_leader.Set(0.5);
  }
  else if (m_controller.GetXButton()){
    rightside_leader.Set(0.5);
  }

    // frc::PWMVictorSPX victor{0}; 
    // victor.Set(0.5);
    // m_left.Set(0.05);
    // testFunction();
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
  frc::DigitalInput ourLimitSwitch{8};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif