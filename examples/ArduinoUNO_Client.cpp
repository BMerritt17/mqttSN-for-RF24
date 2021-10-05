#include <mqttSNmsg.h>

#define NodeID 1 // can be 1-253, 0 is reserved for gateway.




DEVICE_TYPE<DT_NODE> node(9,10);
MSN_MESSAGE<MSN_CONNECT> msgCon;



void event_handler(byte *msg_type, byte *data_buffer)
{
	switch (*msg_type)
	{

	case MSN_ADVERTISE :
		Serial.println("Recieved advertise...");
		break;

	case MSN_CONNACK :
		Serial.println("Recieved connack...");
		Serial.print("Return code: ");
		Serial.println(data_buffer[2]);
		break;

	}

}



void setup() {
	
	Serial.begin(115200);
	
	node.Setup(NodeID); // future work to automate nodeid

	strncpy(msgCon.clientID, "hello node!", CLIENT_ID_SZ);

}



void loop() {

	node.Loop(&event_handler, 10* 1000);

	node.Send(&msgCon, msgCon.msgLength);

	Serial.println("Sent message connect...");

}