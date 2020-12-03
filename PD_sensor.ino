#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9      // [1234] LED를 GPIO 9번 핀에 연결
          // [5678] 그거 아닌거 같은데요?
          // [9012] 1234님 맞습니다.

#define PIN_SERVO 10                    //[3030]SERVO를 10번 핀에 연결
#define PIN_IR A0     //[3037]적외선 센서를 A0에 연결

// Framework setting
#define _DIST_TARGET 255            // [3040] 레일플레이트의 중간지점(목표지점=25.5cm)
#define _DIST_MIN 100                    //[3035] 측정 최소 거리
#define _DIST_MAX 410   //[3036] 측정 가능한 최대 거리


// Distance filter
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35    // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.

// Servo range 
#define _DUTY_MIN 1330                 //[3028] 서보 각도 최소값
#define _DUTY_NEU 1480   //[3038] 레일 수평 서보 펄스폭
#define _DUTY_MAX 1630    //[3031] 서보 최대값

// Servo speed control
#define _SERVO_ANGLE 30   //[3030] servo angle limit 실제 서보의 동작크기
          //[3023] 서보모터의 작동 범위(단위 : degree)
#define _SERVO_SPEED 120            // [3040] 서보의 각속도(초당 각도 변화량)

// Event periods
#define _INTERVAL_DIST 20   //[3039]적외선 센서 측정 간격
#define _INTERVAL_SERVO 20         //[3046]서보갱신간격
#define _INTERVAL_SERIAL 100       //[3030]시리얼 플로터 갱신간격

// PID parameters
#define _KP 1.3 // 비례 제어의 상수 값
#define _KD 54 // 미분 제어의 상수 값

//////////////////////
// global variables //
//////////////////////

// Servo instance     //[3046]서보간격
Servo myservo;                      //[3030]servo

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;     //[3034]적외선센서로 측정한 거리값과 ema필터를 적용한 거리값
int dist_min, dist_max;
float last_result;

// Distance filter
float ema_dist=0;           
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3; 
const float coE[] = {-0.0000082, 0.0044578, 0.6423447, 45.5626413};


// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] interval간격으로 동작 시행을 위한 정수 값
bool event_dist, event_servo, event_serial; //[3023] 적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부


// Servo speed control
int duty_chg_per_interval;  //[3039] interval 당 servo의 돌아가는 최대 정도
int duty_target, duty_curr, duty_neutral;      //[3030]servo 목표 위치, servo 현재 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 
// [3042] 현재 오차값, 이전 오차값, ? , p,i,d 값


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED,OUTPUT);           //[3030]LED를 연결[3027]
  myservo.attach(PIN_SERVO);  //[3039]servo를 연결

// initialize global variables
  dist_min = _DIST_MIN;                      //[3030] 측정값의 최소값
          //[3023] 측정가능 거리의 최소값
  dist_max= _DIST_MAX;    //[3032] 측정값의 최대값
          //[3023] 측정가능 거리의 최대값
  dist_target = _DIST_TARGET;       //[3023] 목표 거리 위치

  duty_neutral = _DUTY_NEU;

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);//[3030]서보를 레일이 수평이 되는 값으로 초기화

// initialize serial port
  Serial.begin(57600);  //[3039] 시리얼 모니터 속도 지정 

// convert angle speed into duty change per interval.
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0); //[3039] 
/*
_SERVO_ANGLE/_SERVO_SPEED=
(_DUTY_MAX - _DUTY_MIN_)* INTERVAL_SERVO/duty_chg_per_interval = 
interval 반복횟수*INTERVAL_SERVO=
_SERVO_ANGLE만큼 돌아가는데 걸리는 시간 
*/

  event_dist = false;
  event_servo = false;
  event_serial = false; 
 
 
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }



////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;    // [3037]
  // get a distance reading from the distance sensor
     float x = filtered_ir_distance(); // [3037]
     float dist_cali = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
     dist_ema = dist_cali;

  // PID control logic
    error_curr = dist_target - dist_ema; //[3034] 목표위치와 실제위치의 오차값 
    pterm = _KP * error_curr; //[3034]
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm;

  // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral + control; //[3034]
    
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX;      
    } else if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN                                               ;
    }

    // update error_prev;
    error_prev = error_curr;
        
    myservo.writeMicroseconds(duty_curr);

 }
  
  if(event_servo) {

    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval 
     if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
      }
      else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
  }//[3034] 서보의 현재 위치가 서보 목표 위치에 도달하기전이면 도달하기전까지 duty_chg_per_interval를 증감시킴. 만약 서보 현재 위치가 서보 목표 위치를 벗어난다면 서보 현재 위치를 서보 목표 위치로 고정

    // update servo position
myservo.writeMicroseconds(duty_curr);//[3034] 

  }
  
  if(event_serial) {
    event_serial = false;               // [3030]
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val; //[3031] 
  const float coE[] = {-0.0000115, 0.0073845, -0.0875637, 80.6927580};
  float volt = float(analogRead(PIN_IR)); //[3031]
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; //[3031]
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;


  return ema_dist; //[3031]
}
