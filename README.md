# FOC_stm32_CAN
PlatformIO project for stm32-bg431b-esc1 bldc motor driver using SimpleFOC and Arduino framework. Command and feedback communciation occurs using CAN bus.


## FOC - Field Oriented Control:
No matter how much I write digging thorugh youtube and internet has no alternative.
Some of the best videos for core concepts:

- https://youtu.be/YPD1_rcXBIE?si=ci6J4SQjhd7sLXG5
  This video and all the video in this series. You'll find links in the description of the video.
- https://youtu.be/Nhy6g9wGHow?si=SNSvLJ6R_x_FJq9U

### SimpleFOC Library:
[SimpleFOC](https://simplefoc.com/) is the most popular arduino library for FOC. It also has a helpful community and the creator himself answers questions regularly. It is very nice and versatile for actually BLDC and Stepper(technically DC motors too) motor control.
  see the sample codes in the website

### stm32-bg431b-esc1
This is a bldc motor driver which have a stm32 g4xx series micro controller inside along with electronics for motor control built in. it has a daughter board(split in half) which is the programmer(stlink-v2) for the micro controller 
* Follow the platformio-project structure of exactly as in the code.
* The platformio.ini file is important to be exactly as it is.
  * The build-flags are important for 1. floating point to be displayed in serial monitor, 2. enabling hardware abstraction layer(HAL) (internal register and op-amp). 3. Enabling the CAN controller.
 
### CAN library for avoid complicated low-level code of stm32 HAL API
The actual implementation of can bus, setting parameters and making it work on stm32 c++ api is complicated. Thanks to (Owen Williams)[https://github.com/owennewo] who has made a simple class which can help us in this regard. I have taken only the stm32 part of his code and put it in the following repo:

- (simplecan_stm32_bg431b_esc1)[https://github.com/InterplanetarCodebase/simplecan_stm32_bg431b_esc1.git]
  This makes it easy to import in a platformio project other than copy pasting.

### This Thread is Gem:
- https://community.simplefoc.com/t/b-g431-esc1-can-interface/2632/1
  This thread in SimpleFOC community is very useful and has answer to most of the questions with simplefoc library and stm32-bg431-esc1

## CAN Bus:
- Best video in my opinion is by Hardwire Electronics: [https://youtu.be/YBrU_eZM110?si=x_pqsI-EEsqhpXmc]
- Standard CAN ID is 11 bit long.(stm32 supports also CANFD which is a upgrade version of the protocol but we won't use it)
- Important thing to know is Lower value IDs(starts with zero) get higher priority. That means between msg.id1=0001011 and msg.id2=0001100 the msg.id2 will be automatically ignored. This is observed by every CAN device and the sender will automatically resend it at a later time(after the ack of this msg).
- CAN message data payload is 8 bytes long.

### CAN application layer:
I had to come up with my own criteria for how the IDs of the CAN msgs would be for differentiating various msgs safely and specifically.

- We have four motors which means four ESCs. Each will send and receive data.
  - Motor TX(ESC to Master MCU): current speed and position value.
  - Motor RC(Master MCU to ESC): command speed.
  - Motor RC(Master MCU to ESC): broadcast empty msg to request for speed and position value.
- Each category has a common ID: see macros `CAN_ID_FILTER_TX`, `CAN_ID_FILTER_RX` and `CAN_DATA_REQ_ID` right shifted by unique motor id.
- Motor IDs are `0 (0b00)`, `1 (0b01)`, `2 (0b10)`, `1 (0b11)`.
- Full ID for RX and TX is 6 bits long e.g. `MOTOR_ID << 4 | CAN_ID_FILTER_TX`
- The broadcast has to be low priority, thus it is a 7 bit long ID which is just a 1 followed by 6 zeros, i.e. `1 << 6`
### Speed and position merging:
Speed and position value is both float value which is 4 bytes and CAN msg payload is 8 bytes. So instead of sending two msgs for two values the speed and angular position is bitmapped into the 8 bytes of data in one payload.
