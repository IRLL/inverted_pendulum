#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>


#define LIN_POS_PIN A0
#define LEFT_LIMIT_SW_PIN 10
#define RIGHT_LIMIT_SW_PIN 11
#define MSG_SIZE 3
#define FREQUENCY 10

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
  next_error_time = 0;

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
	byte left_switch = digitalRead(LEFT_LIMIT_SW_PIN);
	byte right_switch = digitalRead(RIGHT_LIMIT_SW_PIN);
	byte switch_bitfield = 0;
	switch_bitfield |= left_switch ? 0x2 : 0x0;
	switch_bitfield |= right_switch ? 0x1 : 0x0;



	m_msg.data[0] = raw_angle;
	m_msg.data[1] = raw_position;
	m_msg.data[2] = (status_bits << 8) | switch_bitfield;

	m_publisher.publish(&m_msg);
	roshandle.spinOnce();
	delay(50);
}

