
#include <Servo.h>
#include <Stepper.h>
Servo pen_control;
#define pen_pos 0
#define Xstep_mtr 1
#define Ystep_mtr 0
#define up 1
#define down 0
#define step_fwd 1
#define step_rev 0

#define LineDelay 50
#define penDelay 50

#define step_delay 300
#define pulse_delay 300

#define LINE_BUFFER_LENGTH 512

#define Ystep_pin 7
#define Ydir_pin 6
#define Xstep_pin 8
#define Xdir_pin 9
#define Xsense_pin 13
#define Ysense_pin 12
#define servo_pin 5

#define Xstep_max 32000
#define Xstep_min 0
#define Ystep_max 32000
#define Ystep_min 0
#define Zstep_min 0
#define Zstep_max 1

#define  Zmax 1
#define Zmin 0
float Xpos = Xstep_min;
float Ypos = Ystep_min;
float Zpos = Zstep_min;

#define Xstep_sync_val 0
#define Ystep_sync_val 0
#define Xstep_reset_val 5000
#define Ystep_reset_val 5000
#define Xstep_start_val 17000  //change to paper boundaries
#define Ystep_start_val 17000 //cahnge to paper boundaries

float StepInc = 1;
float StepsPerMillimeterX = 7;
float StepsPerMillimeterY = 7;

int Xstep_count = 0;
int Ystep_count = 0;
boolean Xdir = step_fwd;
boolean Ydir = step_fwd;
boolean verbose = false;

const int penZUp = 40;
const int penZDown = 85;

const int stepsPerRevolution = 400;

Stepper Xstepper(stepsPerRevolution, Xstep_pin, Xdir_pin);
Stepper Ystepper(stepsPerRevolution, Ystep_pin, Ydir_pin);

struct point {
  float x;
  float y;
  float z;
};

struct point actuatorPos;

void penUp() {
  pen_control.write(270);
  delay(LineDelay);
  Zpos = Zmax;
  if (verbose) {
    Serial.println("Pen up!");
  }
}
//  Lowers pen
void penDown() {
  pen_control.write(45);
  delay(LineDelay);
  Zpos = Zmin;
  if (verbose) {
    Serial.println("Pen down.");
  }
}

void step_sync()
{
  Serial.println("   SYNCING STEPPER MOTORS _ PLEASE WAIT...");
  boolean Ysen_flag, Xsen_flag;
  while (digitalRead(Ysense_pin) | digitalRead(Xsense_pin)) {
    Ysen_flag = 0; Xsen_flag = 0;

    if (digitalRead(Ysense_pin))
      Ysen_flag = 1;
    if (digitalRead(Xsense_pin))
      Xsen_flag = 1;

    digitalWrite(Ydir_pin, step_rev);
    digitalWrite(Xdir_pin, step_rev);

    digitalWrite(Ystep_pin, Ysen_flag);
    digitalWrite(Xstep_pin, Xsen_flag);

    delayMicroseconds(pulse_delay);

    digitalWrite(Ystep_pin, LOW);
    digitalWrite(Xstep_pin, LOW);

    delayMicroseconds(step_delay);
  }

  //  while(digitalRead(Ysense_pin) | digitalRead(Xsense_pin))
  //  {
  //    Ysen_flag = 1; Xsen_flag = 1;
  //
  //    digitalWrite(Ydir_pin, step_rev);
  //    digitalWrite(Xdir_pin, step_rev);
  //
  //    digitalWrite(Ystep_pin, Ysen_flag);
  //    digitalWrite(Xstep_pin, Xsen_flag);
  //
  //    delayMicroseconds(pulse_delay);
  //
  //    digitalWrite(Ystep_pin, LOW);
  //    digitalWrite(Xstep_pin, LOW);
  //
  //    delayMicroseconds(step_delay);
  //
  //  }



  while (!digitalRead(Ysense_pin) | !digitalRead(Xsense_pin))
  {
    Ysen_flag = 0 ; Xsen_flag = 0;

    if (!digitalRead(Ysense_pin))
      Ysen_flag = 1;
    if (!digitalRead(Xsense_pin))
      Xsen_flag = 1;

    digitalWrite(Ydir_pin, step_fwd);
    digitalWrite(Xdir_pin, step_fwd);

    digitalWrite(Ystep_pin, Ysen_flag);
    digitalWrite(Xstep_pin, Xsen_flag);

    delayMicroseconds(pulse_delay);

    digitalWrite(Ystep_pin, LOW);
    digitalWrite(Xstep_pin, LOW);

    delayMicroseconds(step_delay);
  }

  Xstep_count = Xstep_sync_val; Ystep_count = Ystep_sync_val;

  Serial.println("       - STEPPER MOTORS IN SYNC.....");
  Serial.println("         READY.");
}

void move_one_step(byte step_mtr, bool step_dir)
{
  if (step_mtr == Xstep_mtr)
  {
    if (step_dir == step_fwd)
    {
      if (Xstep_count < Xstep_max)
      {
        digitalWrite(Xdir_pin, step_fwd);
        Xstep_count++;
      }
      else
      {
        Serial.println(" #WARNING - XSTEP MAX LIMIT REACHED# ");
        while (1);
      } //end if xstep_count
    } //end if step_dir

    if (step_dir == step_rev)
    {
      if (Xstep_count > Xstep_min)
      {
        digitalWrite(Xdir_pin, step_rev);
        Xstep_count--;
      }
      else
      {
        Serial.println(" #WARNING - XSTEP MIN LIMIT REACHED# ");
        while (1);
      }
    }
    digitalWrite(Xstep_pin, HIGH);
    delayMicroseconds(pulse_delay);
    digitalWrite(Xstep_pin, LOW);
  }
  ///////////
  if (step_mtr == Ystep_mtr)
  {
    if (step_dir == step_fwd)
    {
      if (Ystep_count < Ystep_max)
      {
        digitalWrite(Ydir_pin, step_fwd);
        Ystep_count++;
      }
      else
      {
        Serial.println(" #WARNING - XSTEP MAX LIMIT REACHED# ");
        while (1);
      } //end if ystep_count
    } //end if step_dir

    if (step_dir == step_rev)
    {
      if (Ystep_count > Ystep_min)
      {
        digitalWrite(Ydir_pin, step_rev);
        Ystep_count--;
      }
      else
      {
        Serial.println(" #WARNING - YSTEP MIN LIMIT REACHED# ");
        while (1);
      }
    }
    digitalWrite(Ystep_pin, HIGH);
    delayMicroseconds(pulse_delay);
    digitalWrite(Ystep_pin, LOW);
  }
  delayMicroseconds(step_delay);
}

void step_goto(int Xpos, int Ypos)
{
  Serial.println("     MOVING STEPPER MOTORS INTO POSITION. PLEASE WAIT...");

  while (Xstep_count < Xpos) {
    move_one_step(Xstep_mtr, step_fwd);
  }
  while (Xstep_count > Xpos) {
    move_one_step(Xstep_mtr, step_rev);
  }
  Serial.println("         - X STEPPER MOTOR IN POSITION...");

  while (Ystep_count < Ypos) {
    move_one_step(Ystep_mtr, step_fwd);
  }
  while (Ystep_count > Ypos) {
    move_one_step(Ystep_mtr, step_rev);
  }
  Serial.println("          - Y STEPPER MOTOR IN POSITION...");
  Serial.println("          READY.");
}

void drawLine(float x1, float y1) {

  if (verbose)
  {
    Serial.print("fx1, fy1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }  

  //  Bring instructions within limits
  if (x1 >= Xstep_max) { 
    x1 = Xstep_max; 
  }
  if (x1 <= Xstep_min) { 
    x1 = Xstep_min; 
  }
  if (y1 >= Ystep_max) { 
    y1 = Ystep_max; 
  }
  if (y1 <= Ystep_min) { 
    y1 = Ystep_min; 
  }

  if (verbose)
  {
    Serial.print("Xpos, Ypos: ");
    Serial.print(Xpos);
    Serial.print(",");
    Serial.print(Ypos);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("x1, y1: ");
    Serial.print(x1);
    Serial.print(",");
    Serial.print(y1);
    Serial.println("");
  }

  //  Convert coordinates to steps
  x1 = (int)(x1*StepsPerMillimeterX);
  y1 = (int)(y1*StepsPerMillimeterY);
  float x0 = Xpos;
  float y0 = Ypos;

  //  Let's find out the change for the coordinates
  long dx = abs(x1-x0);
  long dy = abs(y1-y0);
  int sx = x0<x1 ? StepInc : -StepInc;
  int sy = y0<y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i=0; i<dx; ++i) {
      Xstepper.step(sx);
      //delayMicroseconds(step_delay);
      over+=dy;
      if (over>=dx) {
        over-=dx;
        Ystepper.step(sy);
      //delayMicroseconds(step_delay);
      }
      delayMicroseconds(step_delay);
    }
  }
  else {
    for (i=0; i<dy; ++i) {
      Ystepper.step(sy);
      over+=dx;
      if (over>=dy) {
        over-=dy;
        Xstepper.step(sx);
      }
      delayMicroseconds(step_delay);
    }    
  }

  if (verbose)
  {
    Serial.print("dx, dy:");
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
    Serial.println("");
  }

  if (verbose)
  {
    Serial.print("Going to (");
    Serial.print(x0);
    Serial.print(",");
    Serial.print(y0);
    Serial.println(")");
  }

  //  Delay before any next lines are submitted
  delay(LineDelay);
  //  Update the positions
  Xpos = x1;
  Ypos = y1;
}

void processIncomingLine( char* line, int charNB ) {
  int currentIndex = 0;
  char buffer[ 64 ];                                 // Hope that 64 is enough for 1 parameter
  struct point newPos;

  newPos.x = 0.0;
  newPos.y = 0.0;

  //  Needs to interpret 
  //  G1 for moving
  //  G4 P300 (wait 150ms)
  //  G1 X60 Y30
  //  G1 X30 Y50
  //  M300 S30 (pen down)
  //  M300 S50 (pen up)
  //  Discard anything with a (
  //  Discard any other command!

  while( currentIndex < charNB ) {
    switch ( line[ currentIndex++ ] ) {              // Select command, if any
    case 'U':
      penUp(); 
      break;
    case 'D':
      penDown(); 
      break;
    case 'G':
      buffer[0] = line[ currentIndex++ ];          // /!\ Dirty - Only works with 2 digit commands
      //      buffer[1] = line[ currentIndex++ ];
      //      buffer[2] = '\0';
      buffer[1] = '\0';

      switch ( atoi( buffer ) ){                   // Select G command
      case 0:                                   // G00 & G01 - Movement or fast movement. Same here
      case 1:
        // /!\ Dirty - Suppose that X is before Y
        char* indexX = strchr( line+currentIndex, 'X' );  // Get X/Y position in the string (if any)
        char* indexY = strchr( line+currentIndex, 'Y' );
        if ( indexY <= 0 ) {
          newPos.x = atof( indexX + 1); 
          newPos.y = actuatorPos.y;
        } 
        else if ( indexX <= 0 ) {
          newPos.y = atof( indexY + 1);
          newPos.x = actuatorPos.x;
        } 
        else {
          newPos.y = atof( indexY + 1);
          indexY = '\0';
          newPos.x = atof( indexX + 1);
        }
        drawLine(newPos.x, newPos.y );
        //        Serial.println("ok");
        actuatorPos.x = newPos.x;
        actuatorPos.y = newPos.y;
        break;
      }
      break;
    case 'M':
      buffer[0] = line[ currentIndex++ ];        // /!\ Dirty - Only works with 3 digit commands
      buffer[1] = line[ currentIndex++ ];
      buffer[2] = line[ currentIndex++ ];
      buffer[3] = '\0';
      switch ( atoi( buffer ) ){
      case 300:
        {
          char* indexS = strchr( line+currentIndex, 'S' );
          float Spos = atof( indexS + 1);
          //          Serial.println("ok");
          if (Spos == 30) { 
            penDown(); 
          }
          if (Spos == 50) { 
            penUp(); 
          }
          break;
        }
      case 114:                                // M114 - Repport position
        Serial.print( "Absolute position : X = " );
        Serial.print( actuatorPos.x );
        Serial.print( "  -  Y = " );
        Serial.println( actuatorPos.y );
        break;
      default:
        Serial.print( "Command not recognized : M");
        Serial.println( buffer );
      }
    }
  }



}


void setup() {
  Serial.begin(9600);
  pen_control.attach(servo_pin);
  pinMode(Xstep_pin, OUTPUT);  digitalWrite(Xstep_pin, LOW);
  pinMode(Xdir_pin, OUTPUT);
  pinMode(Xsense_pin, INPUT);
  pinMode(Ystep_pin, OUTPUT);  digitalWrite(Ystep_pin, LOW);
  pinMode(Ydir_pin, OUTPUT);
  pinMode(Ysense_pin, INPUT);
  Serial.println("     ######### WELCOME TO CNC JOTTER ########");
  Serial.println(); delay(500);
  pen_control.write(penZUp);
  delay(200);
  step_sync();
  step_goto(Xstep_reset_val, Ystep_reset_val);
  step_goto(Xstep_start_val, Ystep_start_val);


 
  //draw();

  Serial.println(Xstep_count);
  Serial.println(Ystep_count);

}

void loop() {

  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  while ( Serial.available()>0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') ) {             
        if ( lineIndex > 0 ) {                        
          line[ lineIndex ] = '\0';                   
          if (verbose) { 
            Serial.print( "Received : "); 
            Serial.println( line ); 
          }
          processIncomingLine( line, lineIndex );
          lineIndex = 0;
        } 
        else { 
         
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");    
      } 
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   // Throw away all comment characters
          if ( c == ')' )  lineIsComment = false;     // End of comment. Resume line.
        } 
        else {
          if ( c <= ' ' ) {                           // Throw away whitepace and control characters
          } 
          else if ( c == '/' ) {                    // Block delete not supported. Ignore character.
          } 
          else if ( c == '(' ) {                    // Enable comments flag and ignore all characters until ')' or EOL.
            lineIsComment = true;
          } 
          else if ( c == ';' ) {
            lineSemiColon = true;
          } 
          else if ( lineIndex >= LINE_BUFFER_LENGTH-1 ) {
            Serial.println( "ERROR - lineBuffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          } 
          else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase
            line[ lineIndex++ ] = c-'a'+'A';
          } 
          else {
            line[ lineIndex++ ] = c;
          }
        }
      }
    }
  }
  
  //    step_sync();
  //    step_goto(Xstep_reset_val, Ystep_reset_val);
  //    draw();
  //
  //    Serial.println(Xstep_count);
  //    Serial.println(Ystep_count);
