// Pneumatic ankle IMU_based control 15th March demo
// Abhishek Kashyap

int imu_number = 1;
const int difference_val_length = 10;

#define MAX_PACKET_LEN          (147)   // 8 IMUs: 147 bytes = 1 + 6*8   +   1 + 6*8   +   1 + 6*8
float Euler[8][3];
float previous_val = 0;
float difference;
float difference_values[difference_val_length];
const int HR_RX = A8;



static uint8_t status = 0; /* to track each byte when iterating over the Serial buffer  */
bool read_sensor_data = false;  /* send actuation signal only after IMU data packet processing is complete*/

typedef struct
{
  uint32_t offsetindex;
  uint8_t buf[MAX_PACKET_LEN];
  uint16_t payload_len;
} Packet_t;
void DispData(Packet_t *pkt);
static Packet_t RxPkt;

unsigned currenttime, pretime;

void setup()
{
  delay(2000);
  Serial.begin(9600);
  Serial2.begin(460800);
  
  pinMode (HR_RX, INPUT);  //Signal pin to input
  RxPkt.payload_len = MAX_PACKET_LEN;
 
  currenttime=millis();
  pretime=currenttime;  

  for (int i = 0; i < difference_val_length; i++)
  {
    difference_values[i] = 0.0;
  }
}


/* unpacking the packet byte-wise */
uint32_t unpack_packet(uint8_t current_byte)
{
  switch (status)  // refer to IMU data-sheet
  {
    case 0: //Read 1st Byte: PRE byte
      if (current_byte == 0x5A)
        status = 1;
      else
        status = 0;
      break;
    case 1: //Read 2nd Byte: TYPE byte
      if (current_byte == 0xA5)
        status = 2;
      else
        status = 0;
      break;
    case 2: //Read 3rd Byte: LENGTH byte (not needed)
      status = 3;
      break;
    case 3: //Read 4th Byte: LENGTH byte (not needed)
      status = 4;
      break;
    case 4: //Read 5th Byte: CRC byte (not needed)
      status = 5;
      break;
    case 5: //Read 6th Byte: CRC byte (not needed)
      status = 6;
      break;
    case 6: // Read 7th byte onwards containing useful information: ID and data
      RxPkt.buf[RxPkt.offsetindex++] = current_byte;
      if (RxPkt.offsetindex >= RxPkt.payload_len)
      {
        DispData(&RxPkt);
        status = 0;
        RxPkt.offsetindex = 0;
      }
      break;
    default:
      status = 0;
      break;
  }
}

void DispData(Packet_t *pkt)
{

  if (pkt->buf[0] == 0x72) /* Euler Angle Data  from buf[1] to buf[48] for 8 IMUs */
  {
    // Euler[imu_number][0]  --> Pitch is the actual value multiplied by 100 (datasheet)
    // Euler[imu_number][1]  --> Roll is the actual value multiplied by 100 (datasheet)
    // Euler[imu_number][2]  --> Yaw is the actual value multiplied by 10 (datasheet)

    float pitch_scaling = 0.01, roll_scaling = 0.01, yaw_scaling = 0.1;

    // IMU 0
    Euler[0][0] = ((float)(int16_t)(pkt->buf[1] + (pkt->buf[2] << 8))) * pitch_scaling;
    Euler[0][1] = ((float)(int16_t)(pkt->buf[3] + (pkt->buf[4] << 8))) * roll_scaling;
    Euler[0][2] = ((float)(int16_t)(pkt->buf[5] + (pkt->buf[6] << 8))) * yaw_scaling;

    // IMU 1
    Euler[1][0] = ((float)(int16_t)(pkt->buf[7] + (pkt->buf[8] << 8))) * pitch_scaling;
    Euler[1][1] = ((float)(int16_t)(pkt->buf[9] + (pkt->buf[10] << 8))) * roll_scaling;
    Euler[1][2] = ((float)(int16_t)(pkt->buf[11] + (pkt->buf[12] << 8))) * yaw_scaling;

    // IMU 2
    Euler[2][0] = ((float)(int16_t)(pkt->buf[13] + (pkt->buf[14] << 8))) * pitch_scaling;
    Euler[2][1] = ((float)(int16_t)(pkt->buf[15] + (pkt->buf[16] << 8))) * roll_scaling;
    Euler[2][2] = ((float)(int16_t)(pkt->buf[17] + (pkt->buf[18] << 8))) * yaw_scaling;

    // IMU 3
    Euler[3][0] = ((float)(int16_t)(pkt->buf[19] + (pkt->buf[20] << 8))) * pitch_scaling;
    Euler[3][1] = ((float)(int16_t)(pkt->buf[21] + (pkt->buf[22] << 8))) * roll_scaling;
    Euler[3][2] = ((float)(int16_t)(pkt->buf[23] + (pkt->buf[24] << 8))) * yaw_scaling;

    // IMU 4
    Euler[4][0] = ((float)(int16_t)(pkt->buf[25] + (pkt->buf[26] << 8))) * pitch_scaling;
    Euler[4][1] = ((float)(int16_t)(pkt->buf[27] + (pkt->buf[28] << 8))) * roll_scaling;
    Euler[4][2] = ((float)(int16_t)(pkt->buf[29] + (pkt->buf[30] << 8))) * yaw_scaling;

    // IMU 5
    Euler[5][0] = ((float)(int16_t)(pkt->buf[31] + (pkt->buf[32] << 8))) * pitch_scaling;
    Euler[5][1] = ((float)(int16_t)(pkt->buf[33] + (pkt->buf[34] << 8))) * roll_scaling;
    Euler[5][2] = ((float)(int16_t)(pkt->buf[35] + (pkt->buf[36] << 8))) * yaw_scaling;

    // IMU 6
    Euler[6][0] = ((float)(int16_t)(pkt->buf[37] + (pkt->buf[38] << 8))) * pitch_scaling;
    Euler[6][1] = ((float)(int16_t)(pkt->buf[39] + (pkt->buf[40] << 8))) * roll_scaling;
    Euler[6][2] = ((float)(int16_t)(pkt->buf[41] + (pkt->buf[42] << 8))) * yaw_scaling;

    // IMU 7
    Euler[7][0] = ((float)(int16_t)(pkt->buf[43] + (pkt->buf[44] << 8))) * pitch_scaling;
    Euler[7][1] = ((float)(int16_t)(pkt->buf[45] + (pkt->buf[46] << 8))) * roll_scaling;
    Euler[7][2] = ((float)(int16_t)(pkt->buf[47] + (pkt->buf[48] << 8))) * yaw_scaling;
  }

  // this converts angle from the 0:180 and -180:-0 range to 0:360
  if (Euler[imu_number][1] < 0)
    Euler[imu_number][1] = 360 - abs(Euler[imu_number][1]);
  

  read_sensor_data = true;   // ready to send actuation data signals to pneumatic ankle
}



void loop()
{

  if (Serial2.available())
  {
    uint8_t byte_acquired = Serial2.read();  // reading one byte at a time from the Serial buffer
    unpack_packet(byte_acquired);

    if (read_sensor_data == true)    // (IMU data post-processing is complete)
    {
      //Serial.println(Euler[imu_number][1]);

      // recording last 'difference_val_length' differences
      for (int i = 0; i < difference_val_length-1; i++)
      {
        difference_values[i] = difference_values[i+1];
      }

      difference_values[difference_val_length-1] = Euler[imu_number][1] - previous_val;

      difference = 0;

      // calculate average
      for (int i = 0; i<difference_val_length; i++)
      {
        difference = difference + difference_values[i];
      }
      difference = (difference/difference_val_length)*100;

//      Serial.println(difference);

      if (difference < -70)  // pneumatic muscle should contract 
      {
        Serial.println("10"); 
      }
      else
      {
        Serial.println("0");
      }
      
      previous_val = Euler[imu_number][1];
      read_sensor_data = false;
    }
  }
  
}
