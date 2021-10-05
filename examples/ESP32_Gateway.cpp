#include <mqttSNmsg.h>


DEVICE_TYPE<DT_GATEWAY> gate(4,5);

MSN_MESSAGE<MSN_CONNACK>msgConAck;
MSN_MESSAGE<MSN_ADVERTISE>msgAdv;

void event_handler(byte *msg_type, byte *data_buffer, uint16_t *sender_addr)
{

  	Serial.println("Got mail.");
	Serial.println(*msg_type);
	switch (*msg_type)
	{

	case MSN_CONNECT :

		Serial.println("Connect Msg Rec.");

		msgConAck.returnCode = RC_ACCEPTED;

		gate.SendTo(&msgConAck,*sender_addr);

		Serial.println("Send Conack..");

		break;

	}

}


void setup() {
	
	Serial.begin(115200);
	
	gate.Setup();

	msgAdv.gwID = 0xfe;

}



void loop() {

	gate.Loop(&event_handler, 5 * 1000);

	Serial.println("Loop again...");

	gate.SendToAll(&msgAdv);

}