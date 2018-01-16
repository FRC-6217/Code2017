//#define _GLIBCXX_USE_CXX11_ABI 0
//Include needed libraries
#include <iostream>
#include <memory>
#include <string>
#include <cmath>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include "PIDNumSource.h"
//#include "PIDNumOutput.h"

//Be able to use functions from Autonomous.cpp
#include "Autonomous.h"

/* Arduino control chart
 * | Outputs   |Effect
 * |p3 |p4 |p5 |
 * ___________________
 * | 0 | 0 | 0 | Off
 * | 0 | 0 | 1 | Autonomous
 * | 0 | 1 | 0 | Lined up with peg
 * | 0 | 1 | 1 | Teleop normal
 * | 1 | 0 | 0 |
 * | 1 | 0 | 1 |
 * | 1 | 1 | 0 |
 * | 1 | 1 | 1 |
 */

//Start the class definition
class Robot: public frc::IterativeRobot {
    //Declare the used variables
    frc::Joystick* joystick;
    frc::RobotDrive* robotDrive;
    frc::Joystick* xboxjoystick;
    frc::AnalogInput* ultrasonic;
    frc::AnalogGyro* gyro;
    frc::Encoder* enc;

    frc::VictorSP* winch;
    frc::Spark* shooter;
    frc::Spark* revolver;

    frc::DigitalInput* limitSwitch;

    frc::SendableChooser<const int*>* autoChooser;

    //for communicating with arduino
    frc::DigitalOutput* arduino[3];
    frc::SerialPort* serial;

    //The smart dashboard is broken, and needs some sort of variable passed.
    //These are auto modes.
    const int CROSS = 0;
    const int GEAR_LEFT = 1;
    const int GEAR_CENTER = 2;
    const int GEAR_RIGHT = 3;
    const int BALLS_RED = 4;
    const int BALLS_BLUE = 5;
    const int NOTHING = -1;

    //More variables
    float servoPos;
    int autoState;
    bool debounce;
    bool debounceTwo;
    bool debouncePower;
    bool lockRot;
    int angleOffset;
    int count;
    bool relative;
    double shooterPower;

    int ballNext;

    //These ones are static because the VisionThread is static.
    //It just means they can be accessed from VisionThread.
    static bool actuate;
    static int movement;
    //static bool cameraToggle;
    //static PIDNumSource* visionSource;

    //More PID stuff (Proportion, Integral, Derivative)
    //PIDNumOutput* visionOutput;
    //frc::PIDController* visionControl;

    //variable for vision processing.
    static double values[6];

    public:
    //Startup function
    void RobotInit() {

    	//Allow choosing of vision processing color values
        frc::SmartDashboard::PutNumber("H_Low",0.0);
        frc::SmartDashboard::PutNumber("S_Low",0.0);
        frc::SmartDashboard::PutNumber("V_Low",0.0);
        frc::SmartDashboard::PutNumber("H_High",0.0);
        frc::SmartDashboard::PutNumber("S_High",0.0);
        frc::SmartDashboard::PutNumber("V_High",0.0);

        //Set up the PID loop
        //visionOutput = new PIDNumOutput();
        //visionControl = new frc::PIDController(-0.004, -0.00005, 0.0, visionSource, visionOutput);
        //visionControl->SetInputRange(-0.3,0.3);


        //template is broken, need to use pointers
        //Create options for autonomous mode
        autoChooser = new frc::SendableChooser<const int*>();
        autoChooser->AddDefault("Cross Line", &CROSS);
        autoChooser->AddObject("Left", &GEAR_LEFT);
        autoChooser->AddObject("Center", &GEAR_CENTER);
        autoChooser->AddObject("Right", &GEAR_RIGHT);
        autoChooser->AddObject("Balls (Red)", &BALLS_RED);
        autoChooser->AddObject("Balls (Blue)", &BALLS_BLUE);
        autoChooser->AddObject("Do Nothing", &NOTHING);

        //Put the auto mode chooser on the dashboard
        frc::SmartDashboard::PutData("Auto mode", autoChooser);

        //put a bunch of other junk on the dashboard
        frc::SmartDashboard::PutNumber("Shooter Power", 0.0);
        frc::SmartDashboard::PutString("Drive Mode", "Robot");
        frc::SmartDashboard::PutNumber("Gyro", 0.0);
        frc::SmartDashboard::PutNumber("Encoder", 0.0);
        frc::SmartDashboard::PutNumber("Ultrasonic", 0.0);

        //create the robotDrive object, which controls the drive motors.
        robotDrive = new frc::RobotDrive(0, 1, 2, 3);
        robotDrive->SetInvertedMotor(frc::RobotDrive::MotorType::kFrontRightMotor, true);
        robotDrive->SetInvertedMotor(frc::RobotDrive::MotorType::kRearRightMotor, true);

        robotDrive->SetMaxOutput(0.5);

        //Create objects to reference the joysticks
        joystick = new frc::Joystick(0);
        joystick->SetAxisChannel(Joystick::kTwistAxis, 2);
        xboxjoystick = new frc::Joystick(1);

        //Digital lines to communicate with arduino for lights
        arduino[0] = new DigitalOutput(3);
        arduino[1] = new DigitalOutput(4);
        arduino[2] = new DigitalOutput(5);

        //Doesn't work. Serial Communication with arduino
        //serial = new frc::SerialPort(9600, frc::SerialPort::kUSB1);

        //Start the visionThread function in a different thread.
        //std::thread visionThread(VisionThread);
        //visionThread.detach();

        //create objects for the sensors.
        ultrasonic = new frc::AnalogInput(2);
        gyro = new frc::AnalogGyro(0);
        enc = new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k4X);
        enc->SetDistancePerPulse(-0.0211600227);

        //create objects for the other motors.
        winch = new frc::VictorSP(5);
        shooter = new frc::Spark(6);
        revolver = new frc::Spark(7);

        //A digital input for the gear switch
        limitSwitch = new frc::DigitalInput(2);

        //Debounce variables are used to wait for the release of a button before triggering the effect again.
        debounce = true;
        debounceTwo = true;
        debouncePower = true;

        //lockRot is used to lock the rotation when pressing the line up button.
        //angleOffset stores the current angle, so it can be put back.
        lockRot = false;
        angleOffset = 0;
        relative = false;

        //The power of the shooter.
        shooterPower = 0.6;


        //Used for ball autonomous program, to tell what to do next.
        ballNext = 0;

        //Initialize autonomous class, so that it can access the motors and sensors.
        //Until now, they didn't exist in the scope of auto programs.
        Autonomous::AutoInit(enc, robotDrive, gyro, limitSwitch, shooter, revolver);

        //Set the arduino lights to off. (hopefully). All signals at 0 volts.
        arduino[0]->Set(false);
        arduino[1]->Set(false);
        arduino[2]->Set(false);
    }

    //This function is called at the beginning of the Auto mode.
    void AutonomousInit() override {
    	//Make sure max speed is set, so other speeds are consistent.
        robotDrive->SetMaxOutput(1.0);

        //Turn on the PID loop
        //visionControl->Enable();

        //Reset the sensors
        gyro->Reset();
        enc->Reset();

        //Make sure the auto programs start at the beginning.
        Autonomous::autoState = 0;

        //Make sure to enable the front camera
        //cameraToggle = true;

        //Set the lights
        arduino[0]->Set(false);
        arduino[1]->Set(false);
        arduino[2]->Set(true);
    }

    //This function is called every ~20ms during auto mode.
    void AutonomousPeriodic() {
    	//Set the desired position of the PID loop to be 0
        //visionControl->SetSetpoint(0.0);

        //always stop the winch.
        //EVERY MOTOR MUST BE SET EVERY LOOP. MAKE SURE THIS IS DONE.
        winch->Set(0.0);

        //Calculate the ultrasonic distance
        float distance = ultrasonic->GetValue();
        if (distance < 214.0) {
            distance = 0;
        } else {
            distance -= 214;
        }

        distance *= US_SCALE;
        distance += 10.5;

        //Put the values on the dashboard
        frc::SmartDashboard::PutNumber("Ultrasonic", distance);
        frc::SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
        frc::SmartDashboard::PutNumber("Encoder", enc->GetDistance());
        //frc::SmartDashboard::PutNumber("PID", visionOutput->getValue());

        //Transfer the distance and PID output to the auto functions
        Autonomous::distance = distance;
        //Autonomous::movement = visionOutput->getValue();

        //Get the selected auto program from the dashboard
        const int result = *autoChooser->GetSelected();

        //And run the right auto function based on that. (Remember that this is called repeatedly)
        if (result == CROSS) {
            Autonomous::forward();
        //} else if (result == GEAR_LEFT || *ballNext == 1) {
            Autonomous::baseGearLeft();
        } else if (result == GEAR_CENTER) {
            Autonomous::baseGearCenter();
        //} else if (result == GEAR_RIGHT || *ballNext == 2) {
            Autonomous::baseGearRight();
        } else if (result == BALLS_RED) {
            Autonomous::ballShooter(&ballNext, false);
        } else if (result == BALLS_BLUE) {
        	Autonomous::ballShooter(&ballNext, true);
        } else {
        	//do nothing.
        	shooter->Set(0.0);
        	revolver->Set(0.0);
        	robotDrive->StopMotor();
        }
    }

    //This function is called at the beginning of teleop mode.
    void TeleopInit() {
    	//Stop the PID loop, so it doesn't keep calculation.
        //visionControl->Disable();

        //Count isn't even used anymore. Ignore it.
        count = 0;

        //reset the encoder, and set some variables.
        enc->Reset();
        lockRot = false;
        relative = true;
    }

    void TeleopPeriodic() {
        //visionControl->SetSetpoint(0.0);

        //Put data on the smart dashboard
        frc::SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
        frc::SmartDashboard::PutNumber("Encoder", enc->GetDistance());
        frc::SmartDashboard::PutNumber("Shooter Power", shooterPower);
        //frc::SmartDashboard::PutNumber("PID", visionOutput->getValue());

        //Scale the speed to the throttle on the joystick.
        robotDrive->SetMaxOutput((joystick->GetRawAxis(3) - 1)/-2); //scale speed

        //Update the vision value from the PID loop.
        //Autonomous::movement = visionOutput->getValue();
        //printf("vision: %f\n", visionOutput->getValue());

        //printf("Distance: %i\n", limitSwitch->Get());

        //Add a dead zone, so the robot doesn't move when when you poke the joystick
        float x = fabs(joystick->GetX()) > 0.15 ? joystick->GetX() : 0.0;
        float y = fabs(joystick->GetY()) > 0.1 ? joystick->GetY() : 0.0;
        float twist = fabs(joystick->GetTwist()) > 0.1 ? joystick->GetTwist() / 2 : 0.0;

        //Hold to stay lined up with gear
        if (joystick->GetRawButton(2)) {
            if (!lockRot) {
                lockRot = true;
                angleOffset = gyro->GetAngle() * -1;
                //visionControl->Enable();
            }
            robotDrive->MecanumDrive_Cartesian(Autonomous::movement, y, KP_GYRO * (gyro->GetAngle() + angleOffset));

            //Set the lights
            /*if (visionControl->GetError() < 3) {
                arduino[0]->Set(false);
                arduino[1]->Set(true);
                arduino[2]->Set(false);
            } else {
                arduino[0]->Set(false);
                arduino[1]->Set(true);
                arduino[2]->Set(true);
            }*/
        } else {
        	//set lights
            arduino[0]->Set(false);
            arduino[1]->Set(true);
            arduino[2]->Set(true);
            //visionControl->Disable();
            lockRot = false;

            //Normal driving
            if (!relative) {
                //Move relative to the field
                robotDrive->MecanumDrive_Cartesian(x, y, twist, gyro->GetAngle());
            } else {
                //Move relative to the robot
                robotDrive->MecanumDrive_Cartesian(x, y, twist);
            }
        }

        //Reset the gyro for field-oriented driving
        if (joystick->GetRawButton(7)) {
            gyro->Reset();
        }

        //toggle camera (start on xbox)
        if (xboxjoystick->GetRawButton(8) && debounce) {
            //cameraToggle = !cameraToggle;
            debounce = false;
        } else if (xboxjoystick->GetRawButton(8) == false) {
            debounce = true;
        }

        //toggle drive mode
        if (joystick->GetRawButton(11) && debounceTwo) {
            relative = !relative;
            if (relative) {
                frc::SmartDashboard::PutString("Drive Mode", "Robot");
            } else {
                frc::SmartDashboard::PutString("Drive Mode", "Field");
            }
            debounceTwo = false;
        } else if (joystick->GetRawButton(11) == false) {
            debounceTwo = true;
        }

        //Move winch medium (X)
        if(xboxjoystick->GetRawButton(3)) {
            if (count < 40) {
                winch->Set(0.4);
            } else if (count < 40) {
                winch->Set(0.0);
            } else {
                count = 0;
            }
            count++;
            //move winch fast (A)
        } else if (xboxjoystick->GetRawButton(1)) {
            if (count < 40) {
                winch->Set(0.87);
            } else if (count < 40) {
                winch->Set(0.0);
            } else {
                count = 0;
            }
            count++;
            //move winch very slow (B)
        } else if (xboxjoystick->GetRawButton(2)) {
            winch->Set(0.2);
        } else {
            winch->Set(0.0);
        }

        //spin revolver (left trigger)
        //Trigger is an axis, so test if it is greater than .8
        if (xboxjoystick->GetRawAxis(2) > 0.8) {
            revolver->Set(-0.7);
        } else {
            revolver->Set(0.0);
        }

        //spin the shooter (right trigger)
        if (xboxjoystick->GetRawAxis(3) > 0.8) {
            shooter->Set(shooterPower);
        } else {
            shooter->Set(0);
        }

        //Change the shooter power (RB - up) (LB - down) (Y - reset)
        if (xboxjoystick->GetRawButton(6) && debouncePower) {
            debouncePower = false;
            shooterPower += 0.1;
            if (shooterPower > 1.0) {
                shooterPower = 1.0;
            }
        } else if (xboxjoystick->GetRawButton(5) && debouncePower) {
            debouncePower = false;
            shooterPower -= 0.1;
            if (shooterPower < 0.0) {
                shooterPower = 0.0;
            }
        } else if (xboxjoystick->GetRawButton(4) && debouncePower) {
            debouncePower = false;
            shooterPower = 0.6;
        } else if (xboxjoystick->GetRawButton(6) == false && xboxjoystick->GetRawButton(5) == false && xboxjoystick->GetRawButton(4) == false) {
            debouncePower = true;
        }

        //Set values for vision processing from the dashboard.
        values[0] = frc::SmartDashboard::GetNumber("H_Low",0.0);
        values[1] = frc::SmartDashboard::GetNumber("S_Low",0.0);
        values[2] = frc::SmartDashboard::GetNumber("V_Low",0.0);
        values[3] = frc::SmartDashboard::GetNumber("H_High",0.0);
        values[4] = frc::SmartDashboard::GetNumber("S_High",0.0);
        values[5] = frc::SmartDashboard::GetNumber("V_High",0.0);
    }

    //Code for testing stuff.
    void TestPeriodic() {
        printf("Switch: %d\n", limitSwitch->Get());
        uint8_t data[1];
        char* read = new char[serial->GetBytesReceived()];
        serial->Read(read, serial->GetBytesReceived());
        frc::SmartDashboard::PutString("Serial\n", std::string(read));
    }

    //Turn off lights (don't know if it works)
    void DisabledInit() {
        arduino[0]->Set(false);
        arduino[1]->Set(false);
        arduino[2]->Set(false);
    }

    //This function runs in a separate thread. It processes the camera input.
    /*static void VisionThread()
    {
        //Set up the camera
        int g_exp = 50;

        //First camera
        cs::UsbCamera camera = cs::UsbCamera("usb0",1);
        camera.SetBrightness(5);
        camera.SetExposureManual(g_exp);

        //Second camera
        cs::UsbCamera backCamera = cs::UsbCamera("usb1", 0);
        backCamera.SetBrightness(5);
        backCamera.SetExposureManual(g_exp);
        //frc::SmartDashboard::PutNumber("Brightness", camera.GetBrightness());

        //Start capture
        CameraServer::GetInstance()->StartAutomaticCapture(camera);
        CameraServer::GetInstance()->StartAutomaticCapture(backCamera);

        //Set the resolution low, so not to crash robot again.
        camera.SetResolution(320, 240);
        backCamera.SetResolution(320, 240);

        //Create the objects to get the frames from the camera.
        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(camera);
        cs::CvSink backSink = CameraServer::GetInstance()->GetVideo(backCamera);

        //Create an output to the driver station
        cs::CvSource outputStreamStd =  CameraServer::GetInstance()->PutVideo("Output", 320, 240);

        //Various image objects used in processing.
        cv::Mat source;
        cv::Mat hsv;

        cv::Mat threshOutput;
        cv::Mat out1;
        cv::Mat out2;

        //arrays of contours (groups of pixels)
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        //main vision loop
        while(true) {
        	//if using the front camera:
            if (cameraToggle) {
            	//Get the current frame.
                cvSink.GrabFrame(source);

                //convert from Blue Green Red colr to Hue Saturation Value
                cvtColor(source, hsv, cv::COLOR_BGR2HSV);
                //cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 2, 2);


                //Find green

                //uncomment this line, and comment out the next one, to use values from the dashboard
                //cv::inRange(hsv, cv::Scalar(values[0],values[1],values[2]), cv::Scalar(values[3],values[4],values[5]), threshOutput);
                cv::inRange(hsv, cv::Scalar(70,160,220), cv::Scalar(90,250,255), threshOutput);

                //group nearby pixels into contours
                cv::findContours(threshOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

                //Create some lists for circles and stuff
                std::vector<std::vector<cv::Point>> contours_poly (contours.size());
                std::vector<cv::Point2f> center(contours.size());
                std::vector<float> radius(contours.size());

                std::vector<cv::Point2f> centerLarge;
                std::vector<float> radiusLarge;

                //Set up sme variable for finding the topmost circles. maxY and secY will contain the Y coordinates
                //of the top two circles so far. maxIndex and secIndex will be the index they are in the array.
                int maxY = source.cols;
                int secY = source.cols;
                int maxIndex = -1;
                int secIndex = -1;
                //Create a circle around contours, by looping through each one.
                for( unsigned int i = 0; i < contours.size(); i++ ) {
                    cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
                    minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);

                    //If they are big enough, draw them and find the top two.
                    if (radius[i] > 10.0) {
                        if (center[i].y < maxY) {
                            secY = maxY;
                            secIndex = maxIndex;
                            maxY = center[i].y;
                            maxIndex = i;
                        } else if (center[i].y < secY) {
                            secY = center[i].y;
                            secIndex = i;
                        }
                        cv::Scalar color = cv::Scalar(50, 100, 200);
                        //cv::drawContours(source, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                        cv::drawContours(source, contours_poly, i, color, 1, 8, hierarchy, 0, cv::Point());
                        //cv::rectangle(source, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
                    }
                }

                //draw a line down the middle.
                cv::line(source, cv::Point(source.cols/2,0), cv::Point(source.cols/2,source.rows), cv::Scalar(0,0,255), 1);

                //Find the average of the x coordinates of the largest two circles.
                cv::Mat mean;
                int width = source.cols;
                int pixelCenter = width / 2;

                //only process if there are two circles found.
                if (maxIndex > -1 && secIndex > -1) {
                    centerLarge.push_back(center[maxIndex]);
                    centerLarge.push_back(center[secIndex]);
                    //draw the circles.
                    cv::circle(source, center[maxIndex], (int)radius[maxIndex], cv::Scalar(50, 100, 200), 2, 8, 0);
                    cv::circle(source, center[secIndex], (int)radius[secIndex], cv::Scalar(50, 100, 200), 2, 8, 0);

                    //This function finds the average.
                    cv::reduce(centerLarge, mean, 1, cv::ReduceTypes::REDUCE_AVG);

                    //This creates converts the matrix to a point, and draws a dot.
                    cv::Point2f meanPoint(mean.at<float>(0,0), mean.at<float>(0,1));
                    cv::circle(source, meanPoint, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

                    //Set the input to the PID to the current offset.
                    visionSource->setInput((double)meanPoint.x - (double)pixelCenter);

                } else {
                	//If there are not two circles, stop all movement, and set the offset to 0.
                    Autonomous::movement = 0.0;
                    visionSource->setInput(0.0);
                }
            } else {
            	//If the other camera is selected, grab a frame from it.
                backSink.GrabFrame(source);
            }

            //Send to driver station
            outputStreamStd.PutFrame(source);
        }
    }*/
};

//Static variables must be defined outside the class.
bool Robot::actuate = false;
int Robot::movement = 0;
//bool Robot::cameraToggle = true;
//PIDNumSource* Robot::visionSource = new PIDNumSource(0.0);
double Robot::values[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

//This macro is part of the library, and does all the stuff to set up all the stuff. Needs to be there.
START_ROBOT_CLASS(Robot)
