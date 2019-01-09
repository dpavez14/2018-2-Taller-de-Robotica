// MAVLink Common: https://mavlink.io/en/messages/common.html
// MAVLink Common Commands: https://mavlink.io/en/messages/common.html#mavlink-commands-mavcmd
// ArduPilot Mission Commands: http://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html (see copter commands, this is a subset of MAVLink Commands implemented in ArduPilot)
// Mission Protocol: https://mavlink.io/en/services/mission.html

#include <checksum.h>
#include <mavlink.h> // Common
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>


/********************* Ultrasound sensors variables *********************/

const int pwPin1 = 5; // Altura (sensor inferior)
const int pwPin2 = 0; // Distancia (sensor frontal)
int sensor_1, altitude, sensor_2, distance;
int min_alt = 16;
int max_alt = 645;

/********************* MAVLink variables *********************/
// Used to follow MAVLink Mission Protocol https://mavlink.io/en/services/mission.html
// And keep the HeartBeats cycle, necessary to keep the MAVLink connection
typedef enum { 
  HEARTBEAT_WAIT, 
  HEARTBEAT_RECEIVED,
  HEARTBEAT_DONE,
  DOWNLOAD_MISSION_COUNT_WAIT,
  DOWNLOAD_MISSION_COUNT_RECEIVED,
  DOWNLOAD_MISSION_ITEM_WAIT,
  DOWNLOAD_MISSION_DONE,
  MISSION_CLEAR_ALL_WAIT,
  MISSION_CLEAR_ALL_DONE,
  UPLOAD_MISSION_REQUEST_WAIT,
  UPLOAD_MISSION_ACK_WAIT,
  UPLOAD_MISSION_DONE
} mav_mission_state;

mav_mission_state mission_state = HEARTBEAT_WAIT;
int current_item = 0; //Current mission item
int mission_count = 0;
mavlink_mission_item_int_t[20] mission_items_list; 
mavlink_mission_item_int_t[20] new_mission_items_list;
int new_mission_count = 0;
int last_pos = 0;

/********************* Ultrasound sensors methods *********************/

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

/********************* MAVLink methods *********************/

void Mav_Send_Heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(0xFF, 0, &msg, MAV_TYPE_QUADROTOR,  MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  mission_state = HEARTBEAT_DONE;
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

void Mav_Avoid_Obstacles(){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  Read_Distance_Sensor();

  if (distance <= 200 && mission_state == HEARTBEAT_DONE){ //2m from the obstacle
    // Download mission, following Mission Protocol https://mavlink.io/en/services/mission.html
    mavlink_msg_mission_request_list_pack(0xFF, 0xBE, &msg, 1, 1, MAV_MISSION_TYPE_MISSION);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
    mission_state = DOWNLOAD_MISSION_COUNT_WAIT;
  }
  if (mission_state == DOWNLOAD_MISSION_DONE){
    // Clear old mission, following Mission Protocol https://mavlink.io/en/services/mission.html
    mavlink_msg_mission_clear_all_pack(0xFF, 0xBE, &msg, 1, 1, MAV_MISSION_TYPE_MISSION);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
    mission_state = MISSION_CLEAR_ALL_WAIT;
  }
  if (mission_state == MISSION_CLEAR_ALL_DONE){
    // Modify the list of waypoints
    // http://ardupilot.org/copter/docs/mission-command-list.html
    // https://mavlink.io/en/messages/common.html#mavlink-commands-mavcmd

    // Add MAV_CMD_MISSION_START to ensure the initialization of the mission once uploaded to the UAV
    mavlink_mission_item_int_t start = mission_items_list[0]; // Copy the necessary parameters (note: the 7 MAV_CMD params are all ignored)
    start.command = MAV_CMD_MISSION_START;
    new_mission_count++

    // Not neccesary, because the RTL don't use the TAKEOFF as I thought
    /*if(mission_items_list[0].command == MAV_CMD_NAV_TAKEOFF){
      new_mission_items_list[new_mission_count] = mission_items_list[0];
      new_mission_items_list[new_mission_count].seq = new_mission_count;
      new_mission_count++;
    }*/
    
    // Climb 2m from the current position
    mavlink_mission_item_int_t climb = mission_items_list[current_item]; // 
    climb.command = MAV_CMD_NAV_WAYPOINT;
    climb.param1 = 1; // hold time in seconds
    climb.x = 0; // current latitude
    climb.y = 0; // current longitude
    climb.z = climb.altitude + 2; // climb 2m
    climb.seq = new_mission_count;
    new_mission_items_list[new_mission_count++] = climb;

    // Continue with the mission
    for(int i = 0; current_item <= last_pos; i++) {
      mavlink_mission_item_int_t item = mission_items_list[current_item];
      new_mission_items_list[new_mission_count++] = item;  
    }

    last_pos = new_mission_count - 1;

    // Upload mission with the new list, following Mission Protocol https://mavlink.io/en/services/mission.html
    // last_post = index of the last element of the list
    mavlink_msg_mission_count_pack(0xFF, 0xBE, &msg, 1, 1, last_pos + 1, MAV_MISSION_TYPE_MISSION);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
  }
}


void Mav_Receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial.available()>0) {
    uint8_t c = Serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: //0
          if (mission_state == HEARTBEAT_WAIT){
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            mission_state == HEARTBEAT_RECEIVED
          }
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
          
        case MAVLINK_MSG_ID_MISSION_COUNT: //44
          if (mission_state == DOWNLOAD_MISSION_COUNT_WAIT){
            mavlink_mission_count_t mc;
            mavlink_msg_mission_count_decode(&msg, &mc);
            mission_count = mc.count;
  
            //Send first request
            mavlink_msg_mission_request_int_pack(0xFF, 0xBE, &msg, 1, 1, last_pos, MAV_MISSION_TYPE_MISSION);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
            
            mission_state == DOWNLOAD_MISSION_ITEM_WAIT;
          }
          
          break;
          
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: //73 use MISSION_ITEM_INT over MISSION_ITEM to avoid/reduce precision errors https://mavlink.io/en/services/mission.html#command_message_type
          if(mission_state == DOWNLOAD_MISSION_ITEM_WAIT){
            mavlink_mission_item_int_t item;
            mavlink_msg_mission_count_decode(&msg, &item);
            mission_items_list[last_pos++] = item;
            
            if(mission_count == last_pos){ //All the mission item where received
              // Send ACK
              mavlink_mission_ack_t ack;
              mavlink_msg_mission_ack_pack(0xFF, 0xBE, &msg, 1, 1, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              Serial.write(buf, len);
              mission_state == DOWNLOAD_MISSION_DONE;
              last_pos--; //To keep as the last index, return to 0 when the items are sent back to the pixfalcon
            } 
            else { //Send next request
              mavlink_msg_mission_request_int_pack(0xFF, 0xBE, &msg, 1, 1, last_pos, MAV_MISSION_TYPE_MISSION);
              uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
              Serial.write(buf, len);
            }
          }
          break;

        case MAVLINK_MSG_ID_MISSION_ACK:
          if(mission_state == MISSION_CLEAR_ALL_WAIT)
            mission_state == MISSION_CLEAR_ALL_DONE;
          break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
          if(mission_state == UPLOAD_MISSION_REQUEST_WAIT){
            mavlink_mission_request_int_t request;
            mavlink_msg_mission_request_int_decode(&msg, &request);
            //Send the requested item
            mavlink_mission_item_int_t item = mission_items_list[request.seq];
            mavlink_msg_mission_item_int_encode(0xFF, 0xBE, &msg, &item);
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            Serial.write(buf, len);
  
            if(last_pos == request.seq){ //Last request
              mission_state == UPLOAD_MISSION_ACK_WAIT;
            }
          }
          break;

        case MAVLINK_MSG_ID_MISSION_ACK:
          if (mission_state == UPLOAD_MISSION_ACK_WAIT){
            reset_mavlink_variables();
            mission_state == UPLOAD_MISSION_DONE;
          }
          break;
          
        default:
          break;
    }
  }
}

/********************* Principal program *********************/

void setup() {
  // MAVLink interface start
  Serial.begin(57600);
  
  // Ultrasound sensors start
  pinMode(pwPin1, INPUT);
  pinMode(pwPin2, INPUT);
}

void loop() {
  Mav_Receive();

  if(mission_state == HEARTBEAT_RECEIVED)
    Mav_Send_Heartbeat();

  if(mission_state == HEARTBEAT_DONE){
    // Send current altitude
    Mav_Send_Current_Altitude();
  
    // Over 150cm try to avoid obstacles (reduce errors of the measurement of the sensors, like detect the floor with the frontal sensor)
    if(altitude > 150){ // Current altitude already obtained by Mav_Send_Current_Altitude()
      Mav_Avoid_Obstacles();
    }
  }
  if(mission_state ==  DOWNLOAD_MISSION_DONE ||  mission_state ==  MISSION_CLEAR_ALL_DONE){
    Mav_Avoid_Obstacles();
  }
  
  //Reset to HEARTBEAT_WAIT if nothing is detected or if the mission upload (with the new waypoints) is already done
  if (mission_state == HEARTBEAT_DONE || mission_state == UPLOAD_MISSION_DONE){
    mission_state = HEARTBEAT_WAIT;
  }

}

void reset_mavlink_variables() {
  current_item = 0;
  mission_count = 0;
  mission_items_list = []; 
  new_mission_items_list = [];
  new_mission_count = 0;
  last_pos = 0;
}
