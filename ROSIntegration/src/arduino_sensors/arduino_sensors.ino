#include <Wire.h>
#include <ros.h>
#include <std_msgs/IntArray>




ros::NodeHandle roshandle;
std_msgs::IntArray m_msg;


double conv;

ros::Publisher m_publisher("raw_sensors", &msg)


void setup() {
  Wire.begin();
  conv = 360.0/4096;
  roshandle.initNode();
  roshandle.advertise(&m_publisher);

}

void loop() {

	byte i;
	byte rx[256];
	byte status_bits;
	int raw_angle;
	double angle;
	bool md;
	bool ml;
	bool mh;

	Wire.beginTransmission(0x36);
	Wire.write(0x0B);
	Wire.endTransmission();

	Wire.requestFrom(0x36, 5);



	while(Wire.available())
	{
		rx[i] = Wire.read();
		++i;
	}


	status_bits = rx[0] & ~(0x07);
	raw_angle = rx[1] << 8 | rx[2];
	angle = ((rx[3] << 8 | rx[4]));
	angle *= conv;

	md = status_bits & (1 << 5);
	ml = status_bits & (1 << 4);
	mh = status_bits & (1 << 3);


	//  Serial.print("md: "); Serial.print(md);
	//  Serial.print(" ml: "); Serial.print(ml);
	//  Serial.print(" mh: "); Serial.print(mh);

	if(md)
	{
		Serial.print(" angle: "); Serial.print(angle);
	}
	else
	{
		Serial.print("magnet not detected!");
	}

	Serial.print("\n");


	m_publisher.publish(&m_msg);
	roshandle.spinOnce();
	delay(100);
}
