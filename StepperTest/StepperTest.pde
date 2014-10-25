#include <AccelStepper.h>

#define MAX_X 32000
#define MAX_SPEED 1000
#define ACCELERATION 500
#define STEP_PIN 8
#define DIR_PIN 12

//Init the stepper: 1 says that we are using a Stepper Driver
AccelStepper myStepper(1,STEP_PIN,DIR_PIN);
bool done = false;
void setup ()
{
	Serial.begin(9600);
	//Set an interrupt to run a step if needed
	attachCoreTimerService(runCallback);
	myStepper.setMaxSpeed(MAX_SPEED);
	myStepper.setAcceleration(ACCELERATION);
	Serial.println("Instructions:");
	Serial.println("Use -2 to send the motor back to 0 and -3 to restart.");
}

void loop()
{
	if (Serial.available())
	{
		bool isSpecial = false;
		int newPos = -1;
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
		if (newPos >= 0 && !done)
		{
			myStepper.moveTo(newPos);
		}
		else if (newPos == -2)
		{
			done = true;
			returnToZeroPos();
			//Set up the Enable Here
		}
		else if (newPos == -3)
		{
			done = false;
			//Set up the Enable Here
		}
		else Serial.println("That was not a Valid input!");
	}
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

uint32_t runCallback (uint32_t currentTime)
{
	//If the motor still has to get to the target run more often
	if(myStepper.run()) return (uint32_t) 50;

	//Otherwise save processing interrupts by running less often
	else return (uint32_t) 100;
}

