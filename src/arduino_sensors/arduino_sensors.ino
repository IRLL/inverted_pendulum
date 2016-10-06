#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>


#define LIN_POS_PIN A0
#define MSG_SIZE 3

ros::NodeHandle roshandle;
std_msgs::Int16MultiArray m_msg;
int m_msg_data[MSG_SIZE];

ros::Publisher m_publisher("raw_sensors", &m_msg);



void setup() {
  Wire.begin();

  roshandle.initNode();
  roshandle.advertise(m_publisher);

  m_msg.data = m_msg_data;
  m_msg.data_length = MSG_SIZE;

}

void loop() {

	byte i = 0;
	byte rx[10];
	byte status_bits = 0;
	int raw_angle;

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
	raw_angle = ((rx[3] << 8 | rx[4]));

	int raw_position = analogRead(LIN_POS_PIN);



	m_msg.data[0] = raw_angle;
	m_msg.data[1] = raw_position;
	m_msg.data[2] = (status_bits << 8);

	m_publisher.publish(&m_msg);
	roshandle.spinOnce();
  delay(10);
}

