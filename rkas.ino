/**
 * RF24NetworkHeader header 여기서 header는 통신에서 보충되는 정보들이 들어있는거임.
 * 내가 헤드라고 부르던거는 헤드가 아니라 master였음. ㅈㅅㅈㅅ
 * 실제 데이터는 payload 또는 body 또는 data라고 불림.
 *
 * 전송을 하거나 메세지를 받을 때, LED가 켜진다.
 *
 * 프로그램 순서
 * 1. 셋업
 * 2. 네트워크 업데이트.
 * 3. 부저가 켜져있으면 5 초이상 켜져있었는지 확인하고 끄던지 놔두던지 결정한다.
 * 4. 전송을 했었다면 마지막 전송한지 0.5 초 지났는지 확인하고 sent를 결정한다.
 * 5. 받을 메세지가 있는지 확인한다. 메세지가 있으면 확인하고 부저를 킨다.
 * 6. 거리를 측정한다.
 * 7. 마지막 전송한지 0.5초가 지났다면 거리를 THRESHOLD와 비교한 후 전송을 할지말지 결정한다.
 */
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <SharpIR.h>
#include <Servo.h>

#define MODEL 1080           // IR센서 모델
#define THRESHOLD 25         // 적외선 센서의 측정 경계값, THRESHOLD보다 작으면 검출
#define BUZZER_DURATION 5000 // 부저가 켜져있을 시간
#define SERVO_MAX_POS 180    // 서보 최댓값
#define SERVO_MIN_POS 0      // 서보 최소값
#define SERVO_DURATION 1200  // 서보가 돌아가는데 걸리는 시간
#define SPRAY_PERIOD 10000 // 스프레이를 연속으로 뿌릴 간격
#define SENT_PERIOD 500    // 연속으로 전송할 간격
#define IR_PIN A0
#define CE_PIN 10
#define CS_PIN 9
#define LED_PIN 8
#define TIMER_PIN 7
#define SERVO_PIN 6

// you must change this value
#define NODE_ADDRESS 0      // select an address from the array

SharpIR Sharp(IR_PIN, MODEL);       // IR sensor

Servo spray;                        // servo motor

const uint16_t node_address_set[10] = { 00, 01, 02, 011, 021};

// 0 = Master
// 1-2 (01,02) = Children of Master(00)
// 3   (011)   = Children of (01)
// 4   (021)   = Children of (0)

const uint16_t pos_to_addr_from_this[5][2] = {
    {0123, 0123},           // 0123 is just dummy
    {00, 02},
    {021, 01},
    {01, 021},
    {0123, 011}
};
/**
 *      00      01       011           
 *-------------------------------
 *  <-      <-      <-      <-   
 *-------------------------------
 *  ->      ->      ->      ->   
 *-------------------------------
 *      02      021
 */

// RF pin setting
RF24 radio(CE_PIN, CS_PIN);                                            // CE & CS pins to use (Using 10, 9)
RF24Network network(radio); 


// RF address setiing
const uint16_t this_node = node_address_set[NODE_ADDRESS];             // Our node address
const uint16_t right_node = pos_to_addr_from_this[NODE_ADDRESS][0];    // Right node address of our node
const uint16_t up_right_node = pos_to_addr_from_this[NODE_ADDRESS][1]; // Up right node address of our node


// Buzzer control
unsigned int buzzer_prev = 0;
bool buzzer_status = false;

// Send control
unsigned int last_sent = 0;
bool sent = false;

// Spary control
unsigned int spray_prev = 0;
unsigned int servo_pos = 0;

// Functions
void handle_T(RF24NetworkHeader& header);
void led_on(int led_pin);
void led_off(int led_pin);
void buzzer_on(int buzzer_pin);
void buzzer_off(int buzzer_pin);

void setup(){
    // RF setting
    SPI.begin();                                           // Bring up the RF network
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    network.begin(/*channel*/ 100, /*node address*/ this_node );    // Set channel & address
    
    // Serial setting
    Serial.begin(9600);
    
    // Pin setting
    pinMode(LED_PIN, OUTPUT);       // LED for checking communication is working
    pinMode(TIMER_PIN, OUTPUT);
    
    // Servo setting
    spray.attach(SERVO_PIN);
}

void loop(){
    network.update();       // You have to do it every loop.
    
    if (servo_pos == SERVO_MAX_POS){                              // If servo is turned
        if (millis() - spray_prev > SERVO_DURATION){
            spray.write(SERVO_MIN_POS);
            servo_pos = SERVO_MIN_POS;
        }
    }
    
    if (buzzer_status){                                     // If buzzer is on
        if (millis() - buzzer_prev > BUZZER_DURATION){
            buzzer_off(TIMER_PIN);
            buzzer_status = false;
        }
    }
    
    if (sent){          // if s
        if (millis() - last_sent > SENT_PERIOD){
            sent = false;
        }
    }
    
    while (network.available()){                      // Is there anything ready for us?
        led_on(LED_PIN);
        
        RF24NetworkHeader header;                            // If so, take a look at it
        network.peek(header);
        
        switch (header.type){                              // Dispatch the message to the correct handler.
            case 'T': handle_T(header); break;
            default:
                Serial.print("*** WARNING *** Unknown message type ");
                Serial.println(header.type);
                network.read(header, 0, 0);
                break;
        };
        
        led_off(LED_PIN);
    }
    
    if (!sent){
        bool ok;                                        // Transmit success check.
        unsigned int distance = Sharp.distance();       // Sensing distance for detecting car.
        Serial.print(millis());
        Serial.print(":                             distance is ");
        Serial.println(distance);
        
        if (distance <= THRESHOLD){
            last_sent = millis();
            sent = true;
            led_on(LED_PIN);
            
            ok = send_T(right_node);
            if (ok){                                              // Notify us of the result
                Serial.print(millis());
                Serial.println(": APP Send to right ok");
            }
            else{
                Serial.print(millis());
                Serial.println(": APP Send to right failed");
            }
            
            ok = send_T(up_right_node);
            if (ok){                                              // Notify us of the result
                Serial.print(millis());
                Serial.println(": APP Send to up right ok");
            }
            else{
                Serial.print(millis());
                Serial.println(": APP Send to up right failed");
            }
            
            led_off(LED_PIN);
        }
    }
    delay(50);
}

/**
 * Send a 'T' message, detected
 */
bool send_T(uint16_t to){
    RF24NetworkHeader header(/*to node*/ to, /*type*/ 'T' /*True*/);

    // The 'T' message that we send is just a bool
    bool message = true;
    
    Serial.println("---------------------------------");
    Serial.print(millis());
    Serial.print(": APP Sending ");
    Serial.print(message ? "true" : "false");
    Serial.print(" to ");
    Serial.print(to, OCT);
    Serial.println("...");

    return network.write(header, &message, sizeof(bool));
}

/**
 * Handle a 'T' message
 * Turn on buzzer
 */
void handle_T(RF24NetworkHeader& header){
    bool message;                                                       // The 'T' message is just a bool
    
    network.read(header, &message, sizeof(bool));
    Serial.println("---------------------------------");
    Serial.print(millis());
    Serial.print(": APP Received ");
    Serial.print(message ? "true" : "false");
    Serial.print(" from ");
    Serial.println(header.from_node, OCT);

    if (message){
        // buzzer
        buzzer_on(TIMER_PIN);
        buzzer_status = true;
        buzzer_prev = millis();
        
        // spray
        if (millis() - spray_prev > SPRAY_PERIOD){
            spray.write(SERVO_MAX_POS);
            servo_pos = SERVO_MAX_POS;
            spray_prev = millis();
        }
    }
}

void led_on(int led_pin){
    digitalWrite(led_pin, HIGH);
}

void led_off(int led_pin){
    digitalWrite(led_pin, LOW);
}

void buzzer_on(int timer_pin){
    digitalWrite(timer_pin, HIGH);
}

void buzzer_off(int timer_pin){
    digitalWrite(timer_pin, LOW);
}