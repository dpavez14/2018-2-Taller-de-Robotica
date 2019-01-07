#include <checksum.h>
#include <mavlink.h>
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>

typedef enum { //Used to follow mavlink Mission Protocol https://mavlink.io/en/services/mission.html
  HEARTBEAT_WAIT,
  HEARTBEAT_RECEIVED,
  DOWNLOAD_MISSION_COUNT_WAIT,
  DOWNLOAD_MISSION_COUNT_RECEIVED,
  DOWNLOAD_MISSION_ITEM_WAIT,
  DOWNLOAD_DONE,
  MISSION_CLEAR_ALL_WAIT,
  MISSION_CLEAR_ALL_DONE,
  UPLOAD_MISSION_REQUEST_WAIT,
  UPLOAD_MISSION_ACK_WAIT,
  UPLOAD_MISSION_DONE
} messages_state;

// Sensor variables
const int pwPin1 = 5; // Altura (sensor inferior)
const int pwPin2 = 0; // Distancia (sensor frontal)
int sensor_1, altitude, sensor_2, distance;
int min_alt = 16;
int max_alt = 645;


// Mavlink variables
/*bool stopped = false;
int initial_alt = 0;
int stop_cont = 0;*/
messages_state msg_state = HEARTBEAT_WAIT;
int current_item = 0; //Current mission item
int mission_count = 0:
mavlink_mission_item_int_t[20] mission_items_list; //Max. 20 mission items
int list_pos = 0;

void Read_Altitude_Sensor(){
  sensor_1 = pulseIn(pwPin1, HIGH); // Devuelve distancia en milimetros
  altitude = sensor_1/10;           // Distancia en centimetros
  
  if(altitude < min_alt){ //Asegurar que la altura no es inferior a la minima
    altitude = min_alt;
  }

  if (altitude > max_alt){ //Asegurar que la altura no es superior a la maxima
    altitude = max_alt;
  }
}

void Read_Distance_Sensor(){
  sensor_2 = pulseIn(pwPin2, HIGH); // Devuelve distancia en milimetros
  distance = sensor_2/10;           // Distancia en centimetros
  Serial.println(distance);
}

void Mav_Send_Heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(0xFF, 0, &msg, MAV_TYPE_QUADROTOR,  MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void Mav_Send_Current_Altitude() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Actualizar valor de altitude (global)
  Read_Altitude_Sensor ();

  // Uso de mavlink para rangefinder
  mavlink_msg_distance_sensor_pack(
    0xFF,     // system_id
    0xBE,     // component_id
    &msg,     // msg The MAVLink message to compress the data into
    0,        // (ignored) time_boot_ms [ms]
    min_alt,  // min_distance [cm]
    max_alt,  // max_distance [cm]
    altitude, // current_distance [cm]
    1,        // (ignored) type  MAV_DISTANCE_SENSOR (1: ultrasound)
    0,        // (ignored) id  Onboard ID of the sensor
    25,       // orientation,  MAV_SENSOR_ORIENTATION (downward-facing: 25, forward-facing: 0)
    0         // covariance [cm] 0 for unknown / invalid readings
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}


/*
void Mav_Avoid_Obstacles(){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  Read_Distance_Sensor();

  if (distance <= 100){
    mavlink_mission_current_t current;
    mavlink_msg_mission_current_decode(&msg, &current);
    int actual = current.seq;

    mavlink_mission_count_t count;
    mavlink_msg_mission_count_decode(&msg, &count);
    int cantidad =  count.count;

    mavlink_message_t list;

    mavlink_msg_mission_request_partial_list_pack(0xFF, 0xBE, &list, 1, 1, actual, cantidad, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &list);
    Serial.write(buf, len);

    mavlink_msg_mission_clear_all_pack(0xFF, 0xBE, &msg, 1, 1, 255);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);

  }

  if (!stopped){ // Esta avanzando
    if (distance <= 100){ // Parar


      
      mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 178, 1, 0, 0, 0, 0, 0, 0, 0);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
      
      stopped = true;
    }
    else { // Seguir avanzando
      mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 178, 1, 0, 5, 0, 0, 0, 0, 0);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
    }
  }
  
  // Up
  else if(stopped){ // Esta detenido
    if(distance <= 150){ //Subir mientras la distancia al obstaculo no supere 1.5 metros
      double alt = altitude/100 + 0.5; //Subir medio metro más
      mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 186, 1, alt, MAV_FRAME_MISSION, 0, 0, 0, 0, 0);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);

      if(stop_cont == 0){
        initial_alt = altitude;
      }
      
      stop_cont++;
    }
    
    else { // Avanzar, esperar y bajar
      mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 178, 1, 0, 5, 0, 0, 0, 0, 0);
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);
      
      delay(1000);

      mavlink_message_t msg2;
      uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
      mavlink_msg_command_long_pack(0xFF, 0xBE, &msg2, 1, 1, 186, 1, initial_alt, MAV_FRAME_MISSION, 0, 0, 0, 0, 0); //Altura
      uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &msg2);
      Serial.write(buf2, len2);

      stop_cont = 0;

      stopped = false;
    }
    
  }
}*/

void Mav_Receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial.available()>0) {
    uint8_t c = Serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: //0
          mavlink_heartbeat_t hb;
          mavlink_msg_heartbeat_decode(&msg, &hb);
          //Tal vez implementar un timeout en caso de no recibir este mensage por mucho tiempo
          break;

        //Follow Mission Protocol https://mavlink.io/en/services/mission.html
        case MAVLINK_MSG_ID_MISSION_CURRENT: //42 the current mission item is changed by a message
          mavlink_mission_current_t mc;
          mavlink_msg_mission_current_decode(&msg, &mc);
          current_item = mc.seq;
          break;
          
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: //46 a new mission item is reached
          mavlink_mission_item_reached_t ir;
          mavlink_msg_mission_item_reached_decode(&msg, &ir);
          current_item = ir.seq;
          break;
          
        case MAVLINK_MSG_ID_MISSION_COUNT && msg_state == DOWNLOAD_MISSION_COUNT_WAIT: //44
          mavlink_mission_count_t mc;
          mavlink_msg_mission_count_decode(&msg, &mc);
          mission_count = mc.count;

          //Send first request
          mavlink_msg_mission_request_int_pack(0xFF, 0xBE, &msg, 1, 1, list_pos, MAV_MISSION_TYPE_MISSION);
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          Serial.write(buf, len);
          
          msg_state == DOWNLOAD_MISSION_ITEM_WAIT;
          break;
          
        case MAVLINK_MSG_ID_MISSION_ITEM_INT && msg_state == DOWNLOAD_MISSION_ITEM_WAIT: //73 use MISSION_ITEM_INT over MISSION_ITEM to avoid/reduce precision errors https://mavlink.io/en/services/mission.html#command_message_type
          mavlink_mission_item_int_t item;
          mavlink_msg_mission_count_decode(&msg, &item);
          mission_items_list[list_pos++] = item;
          
          if(mission_count == list_pos){ //All the mission item where received
            // Send ACK
            mavlink_mission_ack_t ack;
            mavlink_msg_mission_ack_pack(0xFF, 0xBE, &msg, 1, 1, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
            msg_state == DOWNLOAD_DONE;
            list_pos--; //To keep as the last index, return to 0 when the items are sent back to the pixfalcon
          } 
          else { //Send next request
            mavlink_msg_mission_request_int_pack(0xFF, 0xBE, &msg, 1, 1, list_pos, MAV_MISSION_TYPE_MISSION);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
          }
          break;

        case MAVLINK_MSG_ID_MISSION_ACK && msg_state == MISSION_CLEAR_ALL_WAIT:
          msg_state == MISSION_CLEAR_ALL_DONE;
          break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_INT && msg_state == UPLOAD_MISSION_REQUEST_WAIT:
          mavlink_mission_request_int_t request;
          mavlink_msg_mission_request_int_decode(&msg, &request);
          //Send the requested item
          mavlink_mission_item_int_t item = mission_items_list[request.seq];
          mavlink_msg_mission_item_int_encode(0xFF, 0xBE, &msg, &item);
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          Serial.write(buf, len);

          if(list_pos == request.seq){ //Last request
            msg_state == UPLOAD_MISSION_ACK_WAIT;
          }
          break;

        case MAVLINK_MSG_ID_MISSION_ACK && msg_state == UPLOAD_MISSION_ACK_WAIT:
          list_pos = 0;
          msg_state == UPLOAD_MISSION_DONE;
          break;
          
        default:
          break;
    }
  }
}


// Launch the serial port in setup
void setup() {
  // MAVLink interface start
  Serial.begin(57600);
  pinMode(pwPin1, INPUT);
  pinMode(pwPin2, INPUT);
}

// Loop your program
void loop() {
  Mav_Receive();

  if(msg_state == HEARTBEAT_RECEIVED)
    Mav_Send_Heartbeat();
  
  // Send current altitude
  Mav_Send_Current_Altitude();

  // Over 50cm try to avoid obstacles
  if(altitude > 50){ // Current altitude already obtained by Mav_Send_Current_Altitude()
    Mav_Avoid_Obstacles();
  }

}
