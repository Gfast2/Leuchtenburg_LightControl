
void StopSong(){
  Message[0] = 0x86;
  Message[1] = 0x01; 
  Message[2] = 0x01;
  Message[3] = 0x00;
  Message[4] = 0x00;
  CRC = crc_8(Message, 6); 

  for (int i=0; i<5; i++){
    MusikSerial.write((byte)Message[i]);
  }
  MusikSerial.write((byte)CRC);


  /*  for (int i=0; i<5; i++){
   Serial.print(Message[i], HEX);
   }
   Serial.print(CRC,HEX);
   Serial.println();
   */
}


void PlaySong(int Song){
  Message[0] = 0x86; //fest Startwert
  Message[1] = 0x01; //Device Address
  Message[2] = 0x01; //command für STOP & Start
  Message[3] = Song; //Song number
  Message[4] = 0x00; //useless parameter 2, default 0
  CRC = crc_8(Message, 6); 

  for (int i=0; i<5; i++){
    MusikSerial.write((byte)Message[i]);
  }
  MusikSerial.write((byte)CRC);

  /*  
   for (int i=0; i<5; i++){
   Serial.print(Message[i], HEX);
   }
   Serial.print(CRC,HEX);
   Serial.println();
   */
}

void SetVolume(int Channel, byte Volume){
  Message[0] = 0x86;
  Message[1] = 0x01;
  Message[2] = 0x02; //command to tune volume
  Message[3] = Channel; // 0-main Volume //1..8-channel volume
  Message[4] = Volume; // set the stark
  CRC = crc_8(Message, 6); 

  for (int i=0; i<5; i++){
    MusikSerial.write((byte)Message[i]);
  }
  MusikSerial.write((byte)CRC);


  /*for (int i=0; i<5; i++){
   Serial.print(Message[i], HEX);
   }
   Serial.print(CRC,HEX);
   Serial.println();
   */

}

void Release() {
  Message[0] = 0x86;
  Message[1] = 0x01;
  Message[2] = 0x00; //command to tune volume
  Message[3] = 0x06; // 0-main Volume //1..8-channel volume
  Message[4] = 0; // set the stark
  CRC = crc_8(Message, 6); 

  for (int i=0; i<5; i++){
    MusikSerial.write((byte)Message[i]);
  }
  MusikSerial.write((byte)CRC);

}

byte crc_8(byte msg[], int len)
{


  byte crc = 0x00;
  byte data;
  boolean flag;
  byte polynom = 0xD5;
  for (int i = 0; i < len; i++)
  {
    data = msg[i];
    for (int bit = 0; bit < 8; bit++)
    {

      if ((crc & 0x80) == 0x80){
        flag = true;
      }
      else flag = false;

      crc <<= 1;

      if ((data & 0x80) == 0x80){
        crc |= (byte)1;
      }

      else {
        crc |= (byte)0;
      }

      //crc |= (data & 0x80) == 0x80 ? (byte)1 : (byte)0;

      data <<= 1;

      if (flag) crc ^= polynom;

    }

  }
  return crc;
}

/*
void Blender(int Channel, int TargetVolume, int BlendspeedSet){
 static char Volume[4];
 static unsigned long BlendTimeOld[4];
 
 if(TargetVolume != Volume[Channel]){
 if(millis() - BlendTimeOld[Channel] > BlendspeedSet){
 int dir = TargetVolume > Volume[Channel] ? 1 : -1; //step size of the Blend
 Volume[Channel] += dir;
 //    SetVolume(Channel, Volume[Channel]);
 BlendTimeOld[Channel] = millis();
 }
 }
 }
 */


int vol_change = 0;
void sound(){ //TODO: move setVolume in else statement, don't let it always send Serial Data. Easy to crash the Musik shiled.
  static int Volume[8];
  static unsigned long BlendTimeOld = 0;
  const int BlendSpeedSet = 300; 

  if(abs(millis() - BlendTimeOld) > BlendSpeedSet){ //abs() to avoid millis() time overflow

    for(int i=0; i<8; i++){

      if( abs(VolumeNow[i]-Volume[i]) > 2) {
        Volume[i] = VolumeNow[i] > Volume[i] ? Volume[i] + SoundBlendSpeed[i] : Volume[i] - SoundBlendSpeed[i];
        SetVolume(i+1, /*VolumeNow[i]*/Volume[i]);  
        vol_change = 0;
      }
      else {
        Volume[i] = VolumeNow[i];
        if(vol_change ==0) { //if this is the first time to go into the targeted volume by this chanel.
          vol_change = 1;
          SetVolume(i+1, /*VolumeNow[i]*/Volume[i]);
        }
      }

      //SetVolume(i+1, /*VolumeNow[i]*/Volume[i]);
      BlendTimeOld = millis();
      //Serial.print(Volume[i]);
      //Serial.print(" ");
    }
    //Serial.println();
  }  
}





