#include <AccelStepper.h>
#include <MsTimer2.h>

#define MAX_SPEED 2000
#define ACCELERATION 20
#define STEP_PIN 12
#define DIR_PIN 8


//Init the stepper: 1 says that we are using a Stepper Driver
AccelStepper myStepper(1,STEP_PIN,DIR_PIN);

void setup ()
{
	Serial.begin(9600);
        //Set an interrupt to run a step if needed
	MsTimer2::set (1, runCallback);
        MsTimer2::start();
        
	myStepper.setMaxSpeed(MAX_SPEED);
	myStepper.setAcceleration(ACCELERATION);
	Serial.println("Instructions:");
	Serial.println("Use -2 to send the motor back to 0 and -3 to restart.");
}

void loop()
{
        myStepper.moveTo(800);
        delay(20000);
        myStepper.moveTo (0);
        delay(20000);
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
	Serial.println("Returning to Home!");
	myStepper.moveTo(0);
	int dist = myStepper.distanceToGo();
	while(dist > 0)
	{
		Serial.print("Steps to Go: ");
		Serial.println(dist, DEC);
	} 
}

void runCallback ()
{
	myStepper.run();
}

