//////////////////////////////////////////////////////////////////////////////////
// AUTHOR: Blake Merritt
// ABOUT: This project creates a MQTT SN service in accordance
// with the MQTT-SN (For Sensor Networks) Protocol Specification.
// https://www.oasis-open.org/committees/download.php/66091/MQTT-SN_spec_v1.2.pdf
//
//////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <mqttSN_config.h>
#include <Arduino.h>
#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>

byte msg_type;
byte data_buffer[MAX_PAYLOAD_SIZE];
uint16_t from_addr;

//  [ MQTT SN FLAG FIELDS ]
// Duplicates 0 if sent first time 1 
// if message retransmited (PUBLISH)
//      DUP_OFF         0b00000000
#define DUP_ON          0b00000001

// Quality Of Service (PUBLISH)
//      QOS_0           0b00000000
#define QOS_1           0b00000100
#define QOS_2           0b00000110

// Retain (PUBLISH)
//      RET_OFF         0b00000000
#define RET_ON          0b00001000

// If set indicates that client is 
// asking for Will Topic and Will 
// Message prompting (CONNECT)
//      WILL_OFF        0b00000000
#define WILL_ON         0b00010000

// Clean Session (CONNECT)
//      CLEAN_OFF       0b00000000
#define CLEAN_ON        0b00100000

// TopicIDType indicates whether 
// message contains a topic id, 
// pre-defined topic id, or 
// short topic name. 
//      NORM_TOPIC_ID   0b00000000
#define PD_TOPIC_ID_ON  0b01000000
#define TOPIC_NAME      0b10000000
//      RESERVED        0b11000000



////// [ MQTT SN MSG TYPES ] //////
enum MSN_MsgType
{
    MSN_ADVERTISE       = 0x00,
    MSN_SEARCHGW        = 0x01,
    MSN_GWINFO          = 0x02,
    
    MSN_CONNECT         = 0x04,
    MSN_CONNACK         = 0x05,
    MSN_WILLTOPICREQ    = 0x06,
    MSN_WILLTOPIC       = 0x07,
    MSN_WILLMSGREQ      = 0x08,
    MSN_WILLMSG         = 0x09,
    MSN_REGISTER        = 0x0A,
    MSN_REGACK          = 0x0B,
    MSN_PUBLISH         = 0x0C,
    MSN_PUBACK          = 0x0D,
    MSN_PUBCOMP         = 0x0E,
    MSN_PUBREC          = 0x0F,
    MSN_PUBREL          = 0x10,

    MSN_SUBSCRIBE       = 0x12,
    MSN_SUBACK          = 0x13,
    MSN_UNSUBSCRIBE     = 0x14,
    MSN_UNSUBACK        = 0x15,
    MSN_PINGREQ         = 0x16,
    MSN_PINGRESP        = 0x17,
    MSN_DISCONNECT      = 0x18,

    MSN_WILLTOPICUPD    = 0x1A,
    MSN_WILLTOPICRESP   = 0x1B,
    MSN_WILLMSGUPD      = 0x1C,
    MSN_WILLMSGRESP     = 0x1D

};


////// [ MQTT SN MSG RETURN CODES ] //////
enum MSN_ReturnCode
{
    RC_ACCEPTED        = 0x00, // Accepted
    RC_REJ_CONGESTED   = 0x01, // Rejected - Congested
    RC_REJ_INV_ID      = 0x02, // Rejected - Invalid ID
    RC_REJ_NOT_SUP     = 0x03  // Rejected - Not Supportedz

};

enum MSN_DeviceType
{
    DT_NODE = 1,
    DT_GATEWAY
};



template <MSN_DeviceType MDT>
class DEVICE_TYPE {};


template<>
class DEVICE_TYPE<DT_GATEWAY>
{

protected:
    RF24 radio;
    RF24Network network;
    RF24Mesh mesh;
   
private:

public:
    DEVICE_TYPE(uint16_t _CE_PIN, uint16_t _CSN_PIN) 
        : radio(RF24(_CE_PIN, _CSN_PIN)), 
            network(radio), mesh(radio, network) {};
    
    bool Setup();
    bool SendTo(void *_Payload, uint16_t _ToAddress);
    void SendToAll(void *_Payload);
    void Loop(void (*event_handler)(byte*, byte*, uint16_t*));
    void Loop(void (*event_handler)(byte*, byte*, uint16_t*), unsigned long _BlockTime);
    void Update();
};

bool DEVICE_TYPE<DT_GATEWAY>::Setup()
{

    mesh.setNodeID(0);

    if(!mesh.begin())
    {
        return false;
    }
    
    mesh.update();

    return true;

}


// mesh.update needs to be called periodically, normal 
// use of mqttSN will handle this, but if users have extended
// delays and or wish to they can call it manually here.
void DEVICE_TYPE<DT_GATEWAY>::Update()
{
    mesh.update();
    mesh.DHCP();
}


void DEVICE_TYPE<DT_GATEWAY>::Loop(void (*event_handler)(byte*, byte*, uint16_t*))
{

    while(1)
    {
        mesh.update();
        mesh.DHCP();

        while (network.available())
        {
            RF24NetworkHeader header;

            network.peek(header);

            switch (header.type)
            {
            case 'M' :
                
                network.read(header, &data_buffer, MAX_PAYLOAD_SIZE);
                
                msg_type = data_buffer[1];
                
                from_addr = header.from_node;
                
                event_handler(&msg_type, data_buffer, &from_addr);

                break;
            
            default:
                break;

            }
         
        }
        
    }
    
}


void DEVICE_TYPE<DT_GATEWAY>::Loop(void (*event_handler)(byte*, byte*, uint16_t*), unsigned long _BlockTime)
{

    unsigned long start_time = millis();

    do
    {
        mesh.update();
        mesh.DHCP();

        while (network.available())
        {
            RF24NetworkHeader header;
            
            network.peek(header);

            switch (header.type)
            {
            case 'M' :
                
                network.read(header, &data_buffer, MAX_PAYLOAD_SIZE);
                
                msg_type = data_buffer[1];

                from_addr = header.from_node;
                
                event_handler(&msg_type, data_buffer, &from_addr);

                break;
            
            default:
                break;
                
            }
            
        }

    } while (millis() - start_time < _BlockTime);
    
}


bool DEVICE_TYPE<DT_GATEWAY>::SendTo(void *_Payload, uint16_t _ToAddress)
{
    mesh.update();
    mesh.DHCP();

    bool msg_sent = false;

    int it = 0;

    RF24NetworkHeader header(_ToAddress, OCT);

    while ( ! msg_sent && (it < MAX_RETRY_COUNT))
    {
        if (network.write(header, _Payload , sizeof(_Payload))) {
            
            msg_sent = true;

            break;
            
        }

        delay(1000);
        it++;          
    }
     
    return msg_sent;

}

void DEVICE_TYPE<DT_GATEWAY>::SendToAll(void *_Payload)
{
    mesh.update();
    mesh.DHCP();

    for (int i= 0; i < mesh.addrListTop; i++)
    {


        int it = 0;

        RF24NetworkHeader header(mesh.addrList[i].address, OCT);

        while (it < MAX_RETRY_COUNT)
        {
            if (network.write(header, _Payload , sizeof(_Payload))) {
                
                break;
                
            }

            delay(1000);
            it++;          
        }

    }    

}






template<>
class DEVICE_TYPE<DT_NODE>
{

protected:
    RF24 radio;
    RF24Network network;
    RF24Mesh mesh;
   
private:

public:
    DEVICE_TYPE(uint16_t _CE_PIN, uint16_t _CSN_PIN) 
        : radio(RF24(_CE_PIN, _CSN_PIN)), 
            network(radio), mesh(radio, network) {};
    
    bool Setup(int _NodeID);
    bool Send(void *_Payload, int _Len);
    void Loop(void (*event_handler)(byte*, byte*));
    void Loop(void (*event_handler)(byte*, byte*), unsigned long _BlockTime);
    void Update();
};


bool DEVICE_TYPE<DT_NODE>::Setup(int _NodeID)
{

    mesh.setNodeID(_NodeID);
    if(!mesh.begin())
    {
        return false;
    }
    
    mesh.update();

    return true;
}


// mesh.update needs to be called periodically, normal 
// use of mqttSN will handle this, but if users have extended
// delays and or wish to they can call it manually here.
void DEVICE_TYPE<DT_NODE>::Update()
{
    mesh.update();
}

void DEVICE_TYPE<DT_NODE>::Loop(void (*event_handler)(byte*, byte*))
{

    while(1)
    {
        mesh.update();

        while (network.available())
        {
            RF24NetworkHeader header;

            network.read(header, &data_buffer, MAX_PAYLOAD_SIZE);

            msg_type = data_buffer[1];

            event_handler(&msg_type, data_buffer);
            
        }
        
    }
    
}


void DEVICE_TYPE<DT_NODE>::Loop(void (*event_handler)(byte*, byte*), unsigned long _BlockTime)
{

    unsigned long start_time = millis();

    do
    {
        mesh.update();

        while (network.available())
        {
            RF24NetworkHeader header;

            network.read(header, &data_buffer, MAX_PAYLOAD_SIZE);

            msg_type = data_buffer[1];

            event_handler(&msg_type, data_buffer);
            
        }


    } while (millis() - start_time < _BlockTime);
    

}

bool DEVICE_TYPE<DT_NODE>::Send(void *_Payload, int _Len)
{
    mesh.update();
    bool msg_sent = false;
    int it = 0;


    while ( ! msg_sent && (it < MAX_RETRY_COUNT))
    {
        if (mesh.write(_Payload, 'M', _Len)) {
            
            msg_sent = true;
            
        } else if ( ! mesh.checkConnection() ) {
            
            if( ! mesh.renewAddress()){

                mesh.begin();

            }


        }

        delay(1000);
        it++;          
    }
     
    return msg_sent;

}







#pragma pack(1)


template<MSN_MsgType M_TYPE>
struct MSN_MESSAGE;

template<>
struct MSN_MESSAGE<MSN_ADVERTISE>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *    (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 5;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_ADVERTISE;

    /*
    *   The GwId field is 1-octet long and uniquely identifies a gateway.
    */
	byte gwID;

    /*
    *    The Duration field is 2-octet long and specifies the duration of a time period in seconds. The maximum value that
    *    can be encoded is approximately 18 hours
    */
    uint16_t duration; 

};

template<>
struct MSN_MESSAGE<MSN_SEARCHGW>
{

    /*
    *    PRE-SET
    *
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *    (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 3;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_SEARCHGW;

    /*
    *    The Radius field is 1-octet long and indicates the value of the broadcast radius. The value 0x00 means “broadcast
    *    to all nodes in the network”
    */
	byte radius; 



};


template<>
struct MSN_MESSAGE<MSN_GWINFO>
{

    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 5;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_GWINFO;

    /*
    *    The GwId field is 1-octet long and uniquely identifies a gateway.
    */
	byte gwID;

    /*
    *    The GwAdd field has a variable length and contains the address of a GW. Its depends on the network over which
    *    MQTT-SN operates and is indicated in the first octet of this field. For example, in a ZigBee network the network
    *    address is 2-octet long.
    */
	uint16_t gwAdd;
	

};

template<>
struct MSN_MESSAGE<MSN_CONNECT>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *    (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 6 + CLIENT_ID_SZ;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_CONNECT;

    /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0X00;

	/*
    *    PRE-SET
    *
    *    The ProtocolId is 1-octet long. It is only present in a CONNECT message and corresponds to the MQTT ‘protocol
    *    name’ and ‘protocol version’.
    *    It is coded 0x01. All other values are reserved.
    */
    byte protoID = 0x01;

    /*
    *    The Duration field is 2-octet long and specifies the duration of a time period in seconds. The maximum value that
    *    can be encoded is approximately 18 hours
    */
    uint16_t duration;

    /*
    *    As with MQTT, the ClientId field has a variable length and contains a 1-23 character long string that uniquely
    *    identifies the client to the server.
    */
    char clientID[CLIENT_ID_SZ];


};

template<>
struct MSN_MESSAGE<MSN_CONNACK>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 3;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_CONNACK;

    /*
    *    The value and meaning of the 1-octet long ReturnCode field is shown in Table 5
    */
	byte returnCode;

};

template<>
struct MSN_MESSAGE<MSN_WILLTOPICREQ>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 2;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLTOPICREQ;


};


template<>
struct MSN_MESSAGE<MSN_WILLTOPIC>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 3 + WILL_TOPIC_SZ;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLTOPIC;

    /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

    /*
    *    The WillTopic field has a variable length and contains the Will topic name.
    */
	char willTopic[WILL_TOPIC_SZ];

};

template<>
struct MSN_MESSAGE<MSN_WILLMSGREQ>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 2;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLMSGREQ;


};


template<>
struct MSN_MESSAGE<MSN_WILLMSG>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 2 + WILL_MSG_SZ;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLMSG;

    /*
    *    The WillMsg field has a variable length and contains the Will message.
    */
	char willMsg[WILL_MSG_SZ];


};

template<>
struct MSN_MESSAGE<MSN_REGISTER>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *    (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 6;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_REGISTER;

	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


};


template<>
struct MSN_MESSAGE<MSN_REGACK>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 7;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_REGACK;

	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;

    /*
    *    The value and meaning of the 1-octet long ReturnCode field is shown in Table 5
    */
	byte returnCode; 

};


template<>
struct MSN_MESSAGE<MSN_PUBLISH>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 7 + PUBLISH_SZ;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PUBLISH;

    /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;

    /*
    *    The Data field corresponds to payload of an MQTT PUBLISH message. It has a variable length and contains the
    *    application data that is being published
    */
	char msgData[PUBLISH_SZ];


};


template<>
struct MSN_MESSAGE<MSN_PUBACK>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 8;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PUBACK;

    /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;

    /*
    *    The value and meaning of the 1-octet long ReturnCode field is shown in Table 5
    */
	byte returnCode; 


};

template<>
struct MSN_MESSAGE<MSN_PUBREC>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 4;
    
    /*
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PUBREC;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


};



template<>
struct MSN_MESSAGE<MSN_PUBREL>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 4;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PUBREL;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


};



template<>
struct MSN_MESSAGE<MSN_PUBCOMP>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 4;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PUBCOMP;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


};


template<>
struct MSN_MESSAGE<MSN_SUBSCRIBE>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 7;
    
    /*
    *   PRE-SET 
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_SUBSCRIBE;

   /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

};

template<>
struct MSN_MESSAGE<MSN_UNSUBSCRIBE>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 7;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_UNSUBSCRIBE;

   /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

};


template<>
struct MSN_MESSAGE<MSN_SUBACK>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 8;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_SUBACK;

   /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

	/*
    *    The TopicId field is 2-octet long and contains the value of the topic id. The values “0x0000” and “0xFFFF” are
    *    reserved and therefore should not be used.
    */
	uint16_t topicID;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;

    /*
    *    The value and meaning of the 1-octet long ReturnCode field is shown in Table 5
    */
	byte returnCode; 


};


template<>
struct MSN_MESSAGE<MSN_UNSUBACK>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 4;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_UNSUBACK;

    /*
    *    The MsgId field is 2-octet long and corresponds to the MQTT ‘Message ID’ parameter. It allows the sender to
    *    match a message with its corresponding acknowledgment.
    */
    uint16_t msgID;


};


template<>
struct MSN_MESSAGE<MSN_PINGREQ>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 2 + CLIENT_ID_SZ;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PINGREQ;

    /*
    *    As with MQTT, the ClientId field has a variable length and contains a 1-23 character long string that uniquely
    *    identifies the client to the server.
    */
    char clientID[CLIENT_ID_SZ];

};


template<>
struct MSN_MESSAGE<MSN_PINGRESP>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 2;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_PINGRESP;

};


template<>
struct MSN_MESSAGE<MSN_DISCONNECT>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 4;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_DISCONNECT;

    /*
    *    The Duration field is 2-octet long and specifies the duration of a time period in seconds. The maximum value that
    *    can be encoded is approximately 18 hours
    */
    uint16_t duration;


};



template<>
struct MSN_MESSAGE<MSN_WILLTOPICUPD>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 3 + WILL_TOPIC_SZ;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLTOPICUPD;

   /*
    *    The Flags field is 1-octet and contains the following flags (see Table 4):
    *    - DUP: same meaning as with MQTT, i.e. set to “0” if message is sent for the first time; set to “1” if
    *    retransmitted (only relevant within PUBLISH messages);
    *    - QoS: meaning as with MQTT for QoS level 0, 1, and 2; set to “0b00” for QoS level 0, “0b01” for QoS level
    *    1, “0b10” for QoS level 2, and “0b11” for new QoS level -1 (only relevant within PUBLISH messages sent
    *    by a client);
    *    - Retain: same meaning as with MQTT (only relevant within PUBLISH messages);
    *    - Will: if set, indicates that client is asking for Will topic and Will message prompting (only relevant within
    *    CONNECT message);
    *    - CleanSession: same meaning as with MQTT, however extended for Will topic and Will message (only
    *    relevant within CONNECT message);
    *    - TopicIdType: indicates whether the field TopicId or TopicName included in this message contains a normal
    *    topic id (set to “0b00”), a pre-defined topic id (set to “0b01”), or a short topic name (set to “0b10”). The
    *    value “0b11” is reserved. Refer to sections 3 and 6.7 for the definition of the various types of topic ids.
    */
    byte flags = 0x00;

    /*
    *    The WillTopic field has a variable length and contains the Will topic name.
    */
	char willTopic[WILL_TOPIC_SZ];

};


template<>
struct MSN_MESSAGE<MSN_WILLMSGUPD>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 2 + WILL_MSG_SZ;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLMSGUPD;

    /*
    *    The WillMsg field has a variable length and contains the Will message.
    */
	char willMsg[WILL_MSG_SZ];

};


template<>
struct MSN_MESSAGE<MSN_WILLTOPICRESP>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 3;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLTOPICRESP;

    /*
    *    The value and meaning of the 1-octet long ReturnCode field is shown in Table 5
    */
	byte returnCode; 

};

template<>
struct MSN_MESSAGE<MSN_WILLMSGRESP>
{
    /*
    *    PRE-SET
    * 
    *    The Length field is either 1 to 3-octets long and specifies the total number of bytes contained in the message
    *   (including the Length field itself).
    *    If the first octet of the Length field is coded “0x01” then the Length field is 3-octet long; in this case, the two
    *    following octets specify the total number of octets of the message (most-significant octet first). Otherwise, the
    *    Length field is only 1-octet long and specifies itself the total number of octets contained in the message.
    *    The 3-octet format allows the encoding of message lengths up to 65535 octets. Messages with lengths smaller
    *    than 256 octets may use the shorter 1-octet format.
    *    Note that because MQTT-SN does not support message fragmentation and reassembly, the maximum message
    *    length that could be used in a network is governed by the maximum packet size that is supported by that network,
    *    and not by the maximum length that could be encoded by MQTT-SN.
    */
    byte msgLength = 3;
    
    /*
    *   PRE-SET
    *
    *   The MsgType field is 1-octet long and specifies the message type
    */
    byte msgType = MSN_WILLMSGRESP;

    /*
    *    The value and meaning of the 1-octet long ReturnCode field is shown in Table 5
    */
	byte returnCode; 

};

#pragma pack(0)


