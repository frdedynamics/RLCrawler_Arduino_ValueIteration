#define DEBUG //Uncomment for debugging mode

//#include <MemoryFree.h>
//#include <pgmStrToRAM.h>
#include "myFcns.h"
//#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
#include <Servo.h>
Servo S1, S2;
Encoder rot_encoder(2, 3); //avoid using pins with LEDs attached

int h,i,j,k=0;
long iteration_cnt = 0;
long oldPosition  = 0;
volatile long newPosition;
long newPosition_hold = 0;
long ms, ms_hold =0;
int vel = 0;
int vel_dt = 10; // dt in milliseconds for differentiate the wheel velocity from the rot_encoder position 
int waitforservos = 70; // wait until the servos moved to new postion in milliseconds

float reward =0;
int action; //up=0; down=1; backwards=2; forwards=3;

int gwS1 = 0; //gridworld state of Servo 1
int gwS2 = 0; //gridworld state of Servo 2
int newgwS1 = 0;
int newgwS2 = 0;
int VigwS1 = 0;
int VigwS2 = 0;
int newVigwS1 = 0;
int newVigwS2 = 0;
int ViAction = 0;
float Vcandidates[4] = {0, 0, 0, 0};
float Vmax = 0;    
int policyentry = 0;
int movedir;
float del;

int V_init = 0; //Multiplicaton Factor to initialize QMATRIX
float e_greed = 90;//93; // factor for  e-greedy action selection in range from 0-99; balance between exploration & exploitation
float e_greed_final = 90; //98
float e_greed_step = 0.015; //value with which the e_greed factor is increased in each iteration. This way exploring at the beginning and exploitation at the end can be achieved.
float gamma = 0.95; // 0.5 at 9states worked 0.7 //discount factor of Q-value of next state
float newRweight = 0.3;

bool go_backwards = false;

#define TWENTYFIVE_STATES // Comment this line out to use 3x3 State space
#ifdef TWENTYFIVE_STATES
#define N_STATES     int n_states[2] = {5, 5}; //{Servo1, Servo2}

#define SERVOPOS     int servopos[5][2] = {{40, 30}, \
                                            {65, 60}, \
                                            {90, 90}, \
                                            {115, 120}, \
                                            {140, 150}}; 
                                          
#define GW2STATE     int gw2state[5][5] = {{0, 1, 2, 3, 4}, \
                                            {5, 6, 7, 8, 9}, \
                                            {10, 11, 12, 13, 14}, \
                                            {15, 16, 17, 18, 19}, \
                                            {20, 21, 22, 23, 24}}; //gridworld indices to state number 

#define VALUETABLE float V[5][5] = {{1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}};
#define REWARDTABLE float R[5][5][4] = {{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
//#define REWARDTABLE float R[5][5][4] = {{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, -100, -100}, {0, 0, 100, -100}, {0, 0, 100, -100}, {0, 0, 100, -100}, {0, 0, 100, -100}}};
#define POLICY int P[5][5] = {{1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}, {1, 1, 1, 1, 1}};
#else
#define N_STATES     int n_states[2] = {3, 3};//{Servo1, Servo2} 
#define SERVOPOS     int servopos[3][2] = {{85, 30}, \
                                           {105, 90}, \
                                           {120, 150}};
#define GW2STATE     int gw2state[3][3] = {{0, 1, 2}, \
                                            {3, 4, 5}, \
                                            {6, 7, 8}};
#define VALUETABLE float V[3][3] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
#define REWARDTABLE float R[3][3][4] = {{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
#define POLICY int P[5][5] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
#endif
N_STATES
SERVOPOS
GW2STATE
VALUETABLE
REWARDTABLE
POLICY


void setup() {  

  S1.attach(6); //range 20 to 180
  S2.attach(5); //range 10 to 150; 90 is 45degree up-forward; 150 is forward-down; 10 is backwards-up 
  Serial1.begin(115200);
  Serial1.println("*******************Start-up****************************");
  
  if(go_backwards)
  {movedir = -1;}
  else
  {movedir = 1;}
  


  long rnd_seed;
  int op;
  //randomSeed(25); //35 good for 9 states

  for (int p = A0; p <= A7; p++)
  {
    op = random(0, 2);
    if (op == 0)
    {
      rnd_seed += analogRead(p);
    }
    else if (op == 1)
    {
      rnd_seed *= analogRead(p);
    }
    randomSeed(abs(rnd_seed));
  }
  Serial1.print("Random Seed: ");
  Serial1.println(abs(rnd_seed));

  for (h = 0; h < n_states[0]; h++){
    for (i = 0; i < n_states[1]; i++){
      V[h][i] = V[h][i] * V_init; //random(1, 10); // Initialize Value table
      P[h][i] = P[h][i] * random(0, 4); // Initialize Policy Table
    }
  }

DEBUG_PRINTLN("Policy:");
  for (h=0; h < (n_states[0]); h++){
    for(i=0;i < n_states[1]; i++){
      DEBUG_PRINT(P[h][i]);
      DEBUG_PRINT(", ");
    }
    DEBUG_PRINTLN();
  }

  gwS1 = random(0, n_states[0]);
  gwS2 = random(0, n_states[1]); //random initial state
  Serial1.print("Random initial state: ");
  Serial1.println(gw2state[gwS1][gwS2]);
  S1.write(servopos[gwS1][0]);
  S2.write(servopos[gwS2][1]);
  delay(2000);
  Serial1.print("g1: "); 
  Serial1.print(gwS1);
  Serial1.print("g2: "); 
  Serial1.println(gwS2);
  newPosition_hold = rot_encoder.read();
}

void loop() {

 
  DEBUG_PRINTLN("****************");
  DEBUG_PRINT("OLD STATE: "); DEBUG_PRINT(gwS1);  DEBUG_PRINT(", "); DEBUG_PRINTLN(gwS2); //DEBUG_PRINTLN(gw2state[gwS1][gwS2]);
  
//Pick an action by e-greedy policy
if (e_greed <= (e_greed_final - e_greed_step)) { //Increase e_greedy value ofer time to exploit more after some learning
        e_greed = e_greed + e_greed_step;
      }
      else {
        e_greed = e_greed_final;
      }
      DEBUG_PRINT("e-Greed: "); DEBUG_PRINTLN(e_greed);
          
      //Exploration
      if (random(0, 100) > e_greed){ 
        action = random(0, 4);
        DEBUG_PRINTLN("e-Greedy: Random Choice");
      }
      else { //Exploitation
        action = P[gwS1][gwS2];
      }
      DEBUG_PRINT("Action: "); DEBUG_PRINTLN(action);

//compute the new state reached by the picked action  
   action2state(gwS1, gwS2, newgwS1, newgwS2, n_states, action); 
      DEBUG_PRINT("NEW STATE: "); DEBUG_PRINT(newgwS1);  DEBUG_PRINT(", "); DEBUG_PRINTLN(newgwS2); //DEBUG_PRINTLN(gw2state[newgwS1][newgwS2]);
    //DEBUG_PRINT("ng1: ");DEBUG_PRINTLN(newgwS1);
    //DEBUG_PRINT("ng2: ");DEBUG_PRINTLN(newgwS2);    
  
  //update the real world system to the new state -> moving the servos
  //reduce servo speed to the end of the movement with a quadratic growing delay between the servo position increments
  k = 0;
  for (i = 1; i <= abs(servopos[newgwS1][0] - servopos[gwS1][0]); i++)
  {
    if (newgwS1 > gwS1)
    {
      S1.write(servopos[gwS1][0] + i);
    }
    else if (newgwS1 < gwS1)
    {
      S1.write(servopos[gwS1][0] - i);
    }

    if (abs(servopos[newgwS1][0] - servopos[gwS1][0]) - i <= 25 && abs(servopos[newgwS1][0] - servopos[gwS1][0]) - i >= 1)
    {
      del = ((25.0 / 25.0) / 25.0) * ((k+5) * (k+5)); // ((final_delay/delta_steps)/delta_steps)*step^2
      k++;
    }
    else
    {
      del = 0;
    }
    delay((int)del);
    //Serial1.println((int)del);
  }

  for (i = 1; i <= abs(servopos[newgwS2][1] - servopos[gwS2][1]); i++)
  {
    if (newgwS2 > gwS2)
    {
      S2.write(servopos[gwS2][1] + i);
    }
    else if (newgwS2 < gwS2)
    {
      S2.write(servopos[gwS2][1] - i);
    }
    del = ((2.0 / abs(servopos[newgwS2][1] - servopos[gwS2][1])) / abs(servopos[newgwS2][1] - servopos[gwS2][1])) * (i * i); // ((final_delay/delta_steps)/delta_steps)*step^2
    delay((int)del);
  }
  ////delay(waitforservos);
delay(100);
//Measure change in Position with the rotary encoder
   k=0;
   ms_hold = millis();
  while(true) // wait and measure as long as the velocity is above "vel"
  {
    ms = millis();
      if(ms >= ms_hold + vel_dt) 
      {
       ms_hold = ms;
       newPosition = rot_encoder.read(); 
       vel = ((newPosition - newPosition_hold) * 1000) / vel_dt;
        //DEBUG_PRINT("Velocity: ");Serial1.println(vel);
        //DEBUG_PRINTLN(newPosition);
       
          if(abs(vel) < 300) 
          {
           newPosition_hold = newPosition;
           //Serial1.println("BREAK");
           break;
          }
          
          newPosition_hold = newPosition;
       }
      k++;
  }

//Observe Reward
  reward = newPosition - oldPosition;
  reward = reward * movedir; // Negate reward when moving backwards is selected
  reward = (reward/100);
  reward = reward * reward * reward;
  reward = reward - 20;
  ///R[gwS1][gwS2][action] = reward;
  R[gwS1][gwS2][action] = (1-newRweight) * R[gwS1][gwS2][action] + newRweight *  reward; // a Table with the immediate rewards is required for Val.It..  
  //So the rewards for actions are stored in a table and updated throug a lowpass filter in each iteration

  DEBUG_PRINT("OLD POS: "); DEBUG_PRINTLN(oldPosition);
  DEBUG_PRINT("NEW POS: "); DEBUG_PRINTLN(newPosition);
  DEBUG_PRINT("REWARD: "); DEBUG_PRINTLN(reward);
  //delay(1000);

  for (i = 0; i < n_states[0]; i++)
  {
    for( j = 0; j < n_states[1]; j++)
    {
      DEBUG_PRINT("{");
      for ( k = 0; k < 4; k++)
      { 
        DEBUG_PRINT(R[i][j][k]);
        if (k < 3){DEBUG_PRINT(", ");}
      }
      DEBUG_PRINT("}, ");
    }
    DEBUG_PRINTLN();
  }

  oldPosition = newPosition;  // Store current position for next iteration

//Value iteration
  for (VigwS1 = 0; VigwS1 < n_states[0] ; VigwS1++){
    for (VigwS2 = 0; VigwS2 < n_states[1]; VigwS2++){

      for (ViAction = 0; ViAction < 4; ViAction++){
        action2state(VigwS1, VigwS2, newVigwS1, newVigwS2, n_states, ViAction);
        Vcandidates[ViAction] =  R[VigwS1][VigwS2][ViAction] + gamma * V[newVigwS1][newVigwS2];
      }

      Vmax = Vcandidates[0];
      policyentry = 0;
      for (i = 1; i < 4; i++){
        if (Vcandidates[i] > Vmax){
          Vmax = Vcandidates[i];
          policyentry = i;
        }     
      }

      V[VigwS1][VigwS2] = Vmax;
      P[VigwS1][VigwS2] = policyentry;
    }
  }

//Store new state for next iteration  
  gwS2 = newgwS2; 
  gwS1 = newgwS1;

  DEBUG_PRINTLN("Value table:");
  for (h=0; h < (n_states[0]); h++){
    for(i=0;i < n_states[1]; i++){
      DEBUG_PRINT(V[h][i]);
      DEBUG_PRINT(", ");
    }
    DEBUG_PRINTLN();
  }

DEBUG_PRINTLN("Policy:");
  for (h=0; h < (n_states[0]); h++){
    for(i=0;i < n_states[1]; i++){
      DEBUG_PRINT(P[h][i]);
      DEBUG_PRINT(", ");
    }
    DEBUG_PRINTLN();
  }

DEBUG_PRINTLN(iteration_cnt); 
iteration_cnt++; 
//delay(1000);
}
