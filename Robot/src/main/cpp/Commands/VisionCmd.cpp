// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/VisionCmd.h"
#include "Robot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

VisionCmd::VisionCmd(): frc::Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void VisionCmd::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void VisionCmd::Execute() {
    // printf("Vision :: Execute()\n");
    if ((Robot::visionNetworkTable->GetNumber("tv", 0.0) == 1.0)&&
        (Robot::visionNetworkTable->GetNumber("tx", 27) <= 5)&& 
            (Robot::visionNetworkTable->GetNumber("tx", 27) >= -5))
    {
        Robot::controllerState1->m_controller.SetRumble(
                frc::GenericHID::RumbleType::kLeftRumble, .9);
    }
    else
    {
        Robot::controllerState1->m_controller.SetRumble(
            frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    }
    
}

// Make this return true when this Command no longer needs to run execute()
bool VisionCmd::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void VisionCmd::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void VisionCmd::Interrupted() {

}