
#include <Servo.h>
Servo pen_control;
#define pen_pos 0
#define Xstep_mtr 1
#define Ystep_mtr 0
#define up 1
#define down 0
#define step_fwd 1
#define step_rev 0

#define step_delay 50
#define pulse_delay 40

#define Ystep_pin 7
#define Ydir_pin 6
#define Xstep_pin 8
#define Xdir_pin 9
#define Xsense_pin 13
#define Ysense_pin 12
#define servo_pin 5

#define Xstep_max 10000000
#define Xstep_min 0
#define Ystep_max 10000000
#define Ystep_min 0

#define Xstep_sync_val 10
#define Ystep_sync_val 10
#define Xstep_reset_val 5000
#define Ystep_reset_val 5000
#define Xstep_start_val 40000  //change to paper boundaries
#define Ystep_start_val 40000 //cahnge to paper boundaries

int Xstep_count = 0;
int Ystep_count = 0;
boolean Xdir = step_fwd;
boolean Ydir = step_fwd;

void pen_up_down(byte up_down)
{

  if (up_down == up)
  {
    pen_control.write(270);
    pen_pos == 270;
  }
  if (up_down == down)
  {
    pen_control.write(45);
    pen_pos == 45;
  }
}

void step_sync()
{
  pen_up_down(up);
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

void draw()
{
  step_goto(Xstep_start_val, Ystep_start_val);

  //Draw box
  pen_up_down(down);

  step_goto(Xstep_count + 10000, Ystep_count);
  step_goto(Xstep_count, Ystep_count + 10000);
  step_goto(Xstep_count - 10000, Ystep_count);
  step_goto(Xstep_count, Ystep_count - 10000);


}


void setup() {
  Serial.begin(115200);
  pen_control.attach(servo_pin);
  pinMode(Xstep_pin, OUTPUT);  digitalWrite(Xstep_pin, LOW);
  pinMode(Xdir_pin, OUTPUT);
  pinMode(Xsense_pin, INPUT);
  pinMode(Ystep_pin, OUTPUT);  digitalWrite(Ystep_pin, LOW);
  pinMode(Ydir_pin, OUTPUT);
  pinMode(Ysense_pin, INPUT);
  Serial.println("     ######### WELCOME TO CNC JOTTER ########");
  Serial.println(); delay(500);
  step_sync();
  step_goto(250000, 250000);
  pen_up_down(down);
  pen_up_down(up);
  pen_up_down(down);
  //draw();

  Serial.println(Xstep_count);
  Serial.println(Ystep_count);


}

void loop() {


  //    step_sync();
  //    step_goto(Xstep_reset_val, Ystep_reset_val);
  //    draw();
  //
  //    Serial.println(Xstep_count);
  //    Serial.println(Ystep_count);

}
