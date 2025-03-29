const byte numChars = 32;
char data[numChars];
boolean newData = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  recvWithStartEndMarkers();
 }

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '[';
    char endMarker = ']';
    char rc;
    // Read serial while we do not see the start marker
    while (Serial.available() && newData == false) {
        rc = Serial.read();
        // handle recieved bytes
        if (recvInProgress == true) {
          // add byte if its not end marker
            if (rc != endMarker) {
                data[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            // end string and set bools when end marker received
            else {
                data[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        // if start marker found begin receival
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    // parse the new instruction
    if (newData){
      parseInstruction();
    }
}


void parseInstruction() {
  // get instruction code
  uint8_t instruction = (uint8_t)data[0] - '0';
  // store instruction value
  char string[3];
  // get first digit on instruction
  string[0] = data[2];
  // get second digit of instruction if it exists, otherwise end string
  if(data[3])
  {
    string[1] = data[3];
  }
  else
  {
    string[1] = '\0';
  }
  // get third digit of instruction if it exists, otherwise end string
  if(data[4] && data[4]!= ',')
  {
    string[2] = data[4];
  }
  else{
    string[2] = '\0';
  }
  //cast to int 
  int value = atoi(string);
  // perform action based on instruction
  followInstruction(instruction, value);
}

void followInstruction(int instrct, int val) {
  switch(instrct) {
    case 0:
    //Code to move forward
      break;
    case 1:
       //Code to move backward
       break;
    case 2:
       //Code to rotate
       break;
     default:
       // handle error
       break;
  }
  // Send identifier byte to mark instruction complete
  Serial.write('!');
  // be ready to look for new data
  newData = false;
}
