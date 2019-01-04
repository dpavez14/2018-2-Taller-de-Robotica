#include <checksum.h>
#include <mavlink.h>
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>

// Sensor variables
const int pwPin1 = 5; // Altura (inferior)
const int pwPin2 = 0; // Distancia (frontal)
int sensor_1, altitude, sensor_2, distance;
int min_alt = 16;
int max_alt = 645;


// Mavlink variables
bool stopped = false;
int initial_alt = 0;
int stop_cont = 0;

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

      if(cont == 0){
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
  // Send current altitude
  Mav_Send_Current_Altitude();

  // Over 150cm try to avoid obstacles
  if(altitude > 150){ // Current altitude already obtained by Mav_Send_Current_Altitude()
    Mav_Avoid_Obstacles();
  }

}
