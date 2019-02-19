// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Commands/FrontLiftCmd.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

FrontLiftCmd::FrontLiftCmd(): frc::Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void FrontLiftCmd::Initialize()
{
    // initialize our local accessors
    liftSubSystem = Robot::frontLift.get();
    controllerState = Robot::controllerState2.get();
    printf("Frontlift command initialized\n");
}

// Called repeatedly when this Command is scheduled to run
void FrontLiftCmd::Execute() 
{
    // Set motor speed based on Y position of right joystick on controller 2
    float motorSpeed = controllerState->GetRightY();

    // we are running the motor @ 100%. This should be the same as the 
    // other lift motors at 25%
    this->liftSubSystem->SetMotorSpeed(motorSpeed);
}

// Make this return true when this Command no longer needs to run execute()
bool FrontLiftCmd::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void FrontLiftCmd::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FrontLiftCmd::Interrupted() {

}
