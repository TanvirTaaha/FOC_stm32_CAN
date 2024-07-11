#include <SPI.h>
#include <mcp2515.h>

#define INTERRUPT_PIN 2
#define CAN_ID_FILTER_TX 0b1010
#define CAN_ID_FILTER_RX 0b1111
#define CAN_DATA_REQ_ID 1 << 6
// #define CAN_DATA_REQ_ID 0b101011

struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(10);
struct can_frame receiving_can_frame;

int motor_index;
float motor_sensor_speed;
float motor_sensor_position;
bool from_motor;
volatile bool interrupt_flag;

union {
  float floats[2];
  uint8_t bytes[8];
} byte_converter;

char c, buffer[10];

float speed;
void request_odom_data(uint8_t motor);
void send_speed_value(uint8_t motor, float _speed);
void irq_can_interrupt() {
  interrupt_flag = true;
}


void setup() {

  Serial.begin(115200);

  while (!Serial)
    ;

  init_can();


  Serial.println("Example: Write to CAN");
}


void loop() {
  // float speed = 2.0;
  // canMsg1.data[0] = 2;
  if (Serial.available() > 1) {
    memset(buffer, 0, sizeof(buffer));
    delay(1);//without delay sometimes characters doesn't get loaded into serial buffer
    Serial.readBytesUntil('\n', buffer, 8);
    Serial.print("buffer:'");
    Serial.print(buffer);
    Serial.println("'");
    Serial.println(buffer[0]);
    if (buffer[0] == 'r' || buffer[0] == 'R') {
      request_odom_data();
    } else {

      c = buffer[0];
      speed = String(buffer + 1).toFloat();


      Serial.print("before branching c:'");
      Serial.print(c);
      Serial.print("', speed:");
      Serial.println(speed);

      send_speed_value(c - 'A', speed);
      delay(100);
      Serial.print("Message1 sent: ");
      Serial.print("speed:");
      Serial.print(speed);
      Serial.print(" ");
      Serial.print("can_id:");
      Serial.println(canMsg1.can_id);
    }
  }

  if (!interrupt_flag && digitalRead(INTERRUPT_PIN) == HIGH) { request_odom_data(); }

  // if (mcp2515.readMessage(&receiving_can_frame) == MCP2515::ERROR_OK) {
  //   motor_index = (receiving_can_frame.can_id >> 4 & 0b11);
  //   from_motor = ((receiving_can_frame.can_id & 0b1111) == CAN_ID_FILTER_TX);

  //   if (from_motor && receiving_can_frame.can_dlc == 8) {
  //     Serial.print("motor:");
  //     Serial.print(motor_index);
  //     *(uint64_t *)byte_converter.bytes = *(uint64_t *)receiving_can_frame.data;
  //     motor_sensor_speed = byte_converter.floats[0];
  //     motor_sensor_position = byte_converter.floats[1];
  //     Serial.print(" speed:");
  //     Serial.print(motor_sensor_speed);
  //     Serial.print(" position:");
  //     Serial.print(motor_sensor_position);
  //     Serial.println();
  //   }
  // }

  process_if_interrupt();
  delay(100);
}

void init_can() {
  mcp2515.reset();

  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), irq_can_interrupt, LOW);

  mcp2515.setFilterMask(MCP2515::MASK0, false, 0);
  mcp2515.setFilter(MCP2515::RXF0, false, CAN_ID_FILTER_TX);
  mcp2515.setNormalMode();
}

void process_if_interrupt() {
  if (interrupt_flag) {
    interrupt_flag = false;

    uint8_t irq = mcp2515.getInterrupts();
    // Serial.print("Interupt mode, irq:");
    // Serial.println(irq);

    if (irq & MCP2515::CANINTF_RX0IF) {
      // Serial.println("RXB0");
      if (mcp2515.readMessage(MCP2515::RXB0, &receiving_can_frame) == MCP2515::ERROR_OK) {
        // frame contains received from RXB0 message
        print_CAN_msg(&receiving_can_frame);
      }
    }
    if (irq & MCP2515::CANINTF_RX1IF) {
      // Serial.println("RXB1");
      if (mcp2515.readMessage(MCP2515::RXB1, &receiving_can_frame) == MCP2515::ERROR_OK) {
        // frame contains received from RXB1 message
        print_CAN_msg(&receiving_can_frame);
      }
    }
    if (!(irq & (MCP2515::CANINTF_RX0IF | MCP2515::CANINTF_RX1IF))) {
      // Serial.println("Clearing interrupts");
      mcp2515.clearInterrupts();
    }
  }
}

void request_odom_data() {
  canMsg2.can_id = CAN_DATA_REQ_ID;  // leading 1 because the priority should be lower for a request so it doesn't interfere with other msgs
  canMsg2.can_dlc = 0;
  uint64_t s = 0;
  *((uint64_t *)canMsg2.data) = s;
  // Serial.print("senging request, can_id:");
  // Serial.print(canMsg2.can_id);
  // Serial.print(", dlc:");
  // Serial.print(canMsg2.can_dlc);
  // Serial.print(", data:");
  // Serial.print(((char *)canMsg2.data));
  // Serial.println();
  bool send_error = mcp2515.sendMessage(&canMsg2) == MCP2515::ERROR_OK;
  // Serial.println(send_error ? "Sent" : "Error");
}

void send_speed_value(uint8_t motor, float _speed) {
  canMsg1.can_id = motor << 4 | CAN_ID_FILTER_RX;
  canMsg1.can_dlc = sizeof(float);
  *((float *)canMsg1.data) = _speed;
  // Serial.print("sending can msg, id:");
  // Serial.print(canMsg1.can_id);
  // Serial.print(", dlc:");
  // Serial.print(canMsg1.can_dlc);
  // Serial.print(", data:");
  // Serial.println(((char *)canMsg1.data));
  bool send_error = mcp2515.sendMessage(&canMsg1) == MCP2515::ERROR_OK;
  // Serial.println(send_error ? "Sent" : "Error");
}

void print_CAN_msg(struct can_frame *frame) {
  motor_index = (frame->can_id >> 4 & 0b11);
  Serial.print("motor:");
  Serial.print(motor_index);

  *(uint64_t *)byte_converter.bytes = *(uint64_t *)frame->data;
  motor_sensor_speed = byte_converter.floats[0];
  motor_sensor_position = byte_converter.floats[1];
  Serial.print(" speed:");
  Serial.print(motor_sensor_speed);
  Serial.print(" position:");
  Serial.print(motor_sensor_position);
  Serial.println();
}
