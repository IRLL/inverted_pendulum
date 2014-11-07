#include <AccelStepper.h>
//#include <MsTimer2.h>
#include <TimerOne.h>

#define MAX_SPEED 8000
#define ACCELERATION 300
#define STEP_PIN 12
#define DIR_PIN 8
#define INTERRUPT_TIME 100 /* 2=500kHz */

//Init the stepper: 1 says that we are using a Stepper Driver
AccelStepper myStepper(1,STEP_PIN,DIR_PIN);

void setup ()
{
        Timer1.initialize();
        Timer1.attachInterrupt(runCallback, INTERRUPT_TIME);
        Timer1.stop();
        //Set an interrupt to run a step if needed
	//MsTimer2::set (1, runCallback);
        //MsTimer2::start();
        
	myStepper.setMaxSpeed(MAX_SPEED);
	myStepper.setAcceleration(ACCELERATION);
        Timer1.stert();
}

void loop()
{
        myStepper.moveTo(3200);
        delay(30000);
        myStepper.moveTo (0);
        delay(30000);
	/*if (Serial.available())
	{
		bool isSpecial = false;
                newPos = 0;
		for (int i = 0; i < 7; ++i)
		{
			char readChar = Serial.read();
			if (readChar >= '0' && readChar <= '9')
			{
				//Accumulate the bytes to a larger int
				newPos = (newPos * 10) + (readChar - '0');
			}
			else if (readChar == '-') isSpecial = true;
			else i = 7;
		}
		if (isSpecial) newPos = -newPos;
		if (newPos > MAX_X)
		{
			newPos = MAX_X; 
		}
		Serial.print("Going to: ");
		Serial.println(newPos, DEC);
		if (newPos >= 0)
		{
			myStepper.moveTo(newPos);
		}
		else if (newPos == -2)
		{
			returnToZeroPos();
		}

		else Serial.println("That was not a Valid input!");
	}*/
}

void returnToZeroPos()
{
	myStepper.moveTo(0);
}

void runCallback ()
{
	myStepper.run();
}

