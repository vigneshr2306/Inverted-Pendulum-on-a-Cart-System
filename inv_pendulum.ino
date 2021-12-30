/*
Code Author: Vignesh Ravikumar
Project Name: Design and control of real-time inverted pendulum system with force-voltage parameter correlation
Microcontroller: STM32F103C8T6
Datasheet reference: https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
*/

unsigned long long previousmillis = 0,previousmillis1 = 0;
unsigned long long previousmillis2 = 0,previousmillis3 = 0;

double x=0.00,theta=0.00,x_dot=0.00,theta_dot=0.00; //declare the state variables ('x' is position)
float t1=0,t2=0,t3=0,t4=0; 
double prev_theta=0.00, prev_x=0.00;
unsigned long pulses=0;
//declare reference variables to find error
double x_ref=4.92, x_error=0.00, x_prev_error=0.00, x_dot_error=0.00, x_tot_error=0.00, theta_tot_error=0.00, theta_ref=180.00, theta_error=0.00, theta_prev_error=0.00, theta_dot_error=0.00, controller_out=0.00;

double K1=4.9983, K2=7.7518, K3=32.1825, K4=11.1775; //Double PD values derived using pole placement technique
int flag=0,limit_switch=0,b=0,c=0;
int duty_cycle=0;
unsigned int duty_cycle1=0;
int m1=PB9; // motor direction pin

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
pinMode(m1,OUTPUT); 
attachInterrupt(PA4,limit,CHANGE);  //limit switch interrupt
 // Datasheet page 392 Encoder Interface
  Serial.begin(115200);
  RCC_BASE->APB2ENR |= (1<<2); // Enable clock to Port A
  
  RCC_BASE->APB2ENR |= (1<<3); // Enable clock to Port B
  
  // PWM Pulse Generation// PB8 as Output PWM
  
  GPIOB_BASE->CRH |= ((1<<0)|(1<<1)|(1<<3)); 
  GPIOB_BASE->CRH &= ~((1<<2)); 
  TIMER4_BASE->CR1 |= 1;
  TIMER4_BASE->CCMR2 |=((1<<5)|(1<<6));
  TIMER4_BASE->CCMR2 &= ~((1<<0)|(1<<1)|(1<<3)|(1<<4)); 
  TIMER4_BASE->CCER |= (1<<8);
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 50000;
  TIMER4_BASE->CNT = 0;
  TIMER4_BASE->CCR3 = 100;
  
// PA0,PA1,PA6,PA7  as INPUT_PULLUP
  GPIOA_BASE->CRL |= ( (1<<3)|(1<<7)|(1<<27)|(1<<31) );
  GPIOA_BASE->CRL &= ~( 1|(1<<1)|(1<<2)|(1<<4)|(1<<5)|(1<<6)|(1<<24)|(1<<25)|(1<<28)|(1<<29)|(1<<26)|(1<<30));
  GPIOA_BASE->ODR |= 1|(1<<1)|(1<<7)|(1<<6);

  //TIMER2_BASE->CR1 = TIMER_CR1_CEN; // Enable Timer
  TIMER2_BASE->CR1 =1;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR |= 2; // Encoder Mode SMS = 011
  TIMER2_BASE->DIER = 0; // Disable Timer Interrupts
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 257; // Encoder Mode Enable
  TIMER2_BASE->CCMR2 = 0;   
  TIMER2_BASE->CCER = 0;
  TIMER2_BASE->PSC = 0;
  TIMER2_BASE->ARR = 43382; // 43382 Pulse Per revolution for position
  TIMER2_BASE->DCR = 0;
  TIMER2_BASE->CNT = 0;
  
  TIMER3_BASE->CR1 =1;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR |= 3; // Encoder Mode SMS = 011
  TIMER3_BASE->DIER = 0; // Disable Timer Interrupts
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 257; // Encoder Mode Enable
  TIMER3_BASE->CCMR2 = 0;   
  TIMER3_BASE->CCER = 0;
  TIMER3_BASE->PSC = 0;
  TIMER3_BASE->ARR = 3999; // 4000 Pulse Per revolution for angle
  TIMER3_BASE->DCR = 0;
  TIMER3_BASE->CNT = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Loop routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

//Process to swing-up the pendulum
if(b==0){
  begin1();
  }
if(c==0 && b==1){
  begin2();
  TIMER3_BASE->CNT = 0;
  }

if(c==1){
//Serial.println(theta);
 theta = ((TIMER3_BASE->CNT)*0.09); // 4000 pulses per revolution
 x = ((TIMER2_BASE->CNT)*0.000223595); // 43382 pulses per revolution
 x_error=x_ref-x;                      
 theta_error= theta_ref-theta;

errorcal(1); //function to calculate the error at 1 millisecond frequency

controller_out =  (x_error*K1)+(x_dot_error*K2*1000)+  (theta_error*K3) + (theta_dot_error*K4*1000); //Double PD equation
duty_cycle = (23.155*controller_out)+2.011; //Parameter correlation (check report for details)
if(duty_cycle>0&&duty_cycle<300){
  duty_cycle=300;
}
if(duty_cycle<0&&duty_cycle>-300){
  duty_cycle=-300;
}
duty_cycle1 = (abs(duty_cycle))*50000/360;
print1(100);
if(theta==180.00){
  flag=1;
}

if (flag==1){
  if(x<0.2){
    digitalWrite(m1,LOW);
    TIMER4_BASE->CCR3 = 25000;
  }
  else if(x>9){
    digitalWrite(m1,HIGH);
    TIMER4_BASE->CCR3 = 25000; 
  }
  else if(duty_cycle>=0){
    digitalWrite(m1,HIGH);
    TIMER4_BASE->CCR3 = duty_cycle1; 
   }
   else if(duty_cycle<0){
    digitalWrite(m1,LOW);
    TIMER4_BASE->CCR3 = duty_cycle1; 
    }
}

}
}
void begin1(){
"""Function to move the cart to the left extreme to begin the swing-up process"""
   digitalWrite(m1,HIGH);
    TIMER4_BASE->CCR3 = 30000; 
    if(limit_switch==1){
      TIMER4_BASE->CCR3 = 0;
      delay(500); 
      TIMER2_BASE->CNT = 0;
      b=1;
      }
  }
void begin2(){
"""Swing-up process"""
      Serial.println(TIMER2_BASE->CNT);
      if(TIMER2_BASE->CNT < 22000){
      digitalWrite(m1,LOW);
      TIMER4_BASE->CCR3 = 40000;
      }
      else{
        TIMER4_BASE->CCR3 = 0;  
        delay(10000);
       
        c=1;
        }
  }

void errorcal(unsigned int delayy){
"""Calculates the error based on sampling frequency
Args: 
    delayy: indicates the sampling frequency
"""
  if ((millis()-previousmillis)>=delayy){
     theta_dot = int ((theta - prev_theta)*1000/delayy);
     x_dot = int (x-prev_x)*1000/(delayy);
    
    x_dot_error= x_error-x_prev_error;
    theta_dot_error=theta_error-theta_prev_error;
    x_tot_error+=x_error;
    theta_tot_error+=theta_error;

prev_theta = theta;
prev_x = x;
x_prev_error = x_error;
theta_prev_error = theta_error;
previousmillis = millis();

  }
}
void print1(unsigned int delayy1){
"""Prints data based on set frequency
Args: 
    delayy1: indicates the frequency at which the data should be printed
"""
if ((millis()-previousmillis1)>=delayy1){
      Serial.print(duty_cycle);
      Serial.print("                      ");
      Serial.print(x);
      Serial.print("                      ");
      Serial.println(theta);
      previousmillis1 = millis();
}
  }

void limit(){
""" Toggles the limit_switch value to 1 when the cart hits the edge of the setup
"""
  limit_switch = 1;
}
