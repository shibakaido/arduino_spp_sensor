// -*- mode:c++; coding:utf-8-unix; -*-

/**
 * Transmitting sensor data and controlling board state with Bluetooth SPP.
 *
 * USB and Bluetooth releated objects are defined in USB_Host_Shield_2.0 library.
 *
 * < NO WARRANTY >
 */
#include <SPP.h>
#include <usbhub.h>

#define LED_RED 2
#define LED_YELLOW 3
#define FAKE_SENSOR A2

#define CLOCK_INTERVAL 250

/**
 * This variable switches its boolean value every time CLOCK_INTERVAL msec elapsed.
 */
boolean clock_edge = false;

USB Usb;
BTD Btd(&Usb);

// DEVICE NAME : "Arduino Fake Sensor"
// PIN : "0000"
SPP SerialBT(&Btd, "Arduino Fake Sensor", "0000");

enum
  {
    NO_COMMAND,
    START_COMMAND,
    STOP_COMMAND
  };

void loop_task();
int command_check();

/**
 * This function will be called from Arduino.
 */
void setup()
{
  Serial.begin(115200);

  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  while(!Serial)
    {
    }

  if(Usb.Init() == -1)
    {
      Serial.print(F("USB initialization failed.\r\n"));

      // halt
      while (1)
        {
        }
    }

  // LED setup
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_YELLOW, 0);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  Serial.println(F("setup() finished"));
}

/**
 * This function will be called from Arduino.
 */
void loop()
{
  // states
  static enum
  {
    IDLE,
    SCAN,
    CONNECTED,
    DATA_ACTIVE
  } state = IDLE;

  // sensor data packet
  static struct
  {
    unsigned char preamble;
    short sensor_value;
  } packet = {0xaa, 0};

  // common task
  loop_task();

  // state machine --------------------
  switch(state)
    {
    case IDLE:

      digitalWrite(LED_RED, 0);
      digitalWrite(LED_YELLOW, 0);

      if(SerialBT.connected)
        {
          Serial.println("Connected.");

          state = CONNECTED;
          break;
        }

      if(Btd.watingForConnection)
        {
          Serial.println("Waiting for connection.");

          state = SCAN;
          break;
        }
      break;

    case SCAN:

      digitalWrite(LED_RED, clock_edge);
      digitalWrite(LED_YELLOW, 0);

      if(SerialBT.connected)
        {
          Serial.println("Connected.");

          state = CONNECTED;
          break;
        }

      if(!(Btd.watingForConnection))
        {
          Serial.println("Quit waiting for connection.");

          state = IDLE;
          break;
        }

      break;

    case CONNECTED:

      digitalWrite(LED_RED, 1);
      digitalWrite(LED_YELLOW, 0);

      if(!(SerialBT.connected))
        {
          Serial.println("Disconnected.");

          state = IDLE;
          break;
        }

      if(START_COMMAND == command_check())
        {
          Serial.println("START DATA.");

          state = DATA_ACTIVE;
          break;
        }

      break;

    case DATA_ACTIVE:

      digitalWrite(LED_RED, 1);
      digitalWrite(LED_YELLOW, 1);

      if(!(SerialBT.connected))
        {
          Serial.println("Disconnected.");

          state = IDLE;
          break;
        }

      if(STOP_COMMAND == command_check())
        {
          Serial.println("STOP DATA.");

          state = CONNECTED;
          break;
        }

      // Send data
      packet.sensor_value = (short)analogRead(FAKE_SENSOR);
      SerialBT.write((byte *)(&packet), 3);
      SerialBT.flush();

      // It seems that we need interval between successive data...
      delay(50);
      break;

    default:
      Serial.println("[BUG]Invalid state!!");

      // halt
      while(1)
        {
        }
    }
}

/**
 * Common task for all states.
 */
void loop_task()
{
  static unsigned long last_edge = millis();

  Usb.Task();

  if(CLOCK_INTERVAL < (millis() - last_edge))
    {
      clock_edge = !clock_edge;
      last_edge = millis();
    }
}

#define MAX_CHAR_PER_CHECK 20

/**
 * Specialized command parser for this sample.
 */
int command_check()
{
  static enum
  {
    PARSE_STATE_IDLE,
    PARSE_STATE_s,
    PARSE_STATE_t,
    PARSE_STATE_a,
    PARSE_STATE_r,
    PARSE_STATE_t_1,
    PARSE_STATE_o,
    PARSE_STATE_p
  } parse_state = PARSE_STATE_IDLE;

  int recv = -1;
  int read_cnt = 0;
  while(1)
    {
      // When long input occurred, break in order that common task could perform.
      // Parser state will be preserved until next check.
      if(MAX_CHAR_PER_CHECK == read_cnt++)
        {
          break;
        }

      recv = -1;
      if(-1 == (recv = SerialBT.read()))
        {
          break;
        }

      switch(parse_state)
        {
        case PARSE_STATE_IDLE:

          if('s' == recv)
            {
              parse_state = PARSE_STATE_s;
            }
          break;

        case PARSE_STATE_s:

          switch(recv)
            {
            case 's':
              // STAY this state.
              break;
            case 't':
              parse_state = PARSE_STATE_t;
              break;
            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        case PARSE_STATE_t:

          switch(recv)
            {
            case 's':
              parse_state = PARSE_STATE_s;
              break;
            case 'a':
              parse_state = PARSE_STATE_a;
              break;
            case 'o':
              parse_state = PARSE_STATE_o;
              break;
            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        case PARSE_STATE_a:

          switch(recv)
            {
            case 's':
              parse_state = PARSE_STATE_s;
              break;
            case 'r':
              parse_state = PARSE_STATE_r;
              break;
            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        case PARSE_STATE_r:

          switch(recv)
            {
            case 's':
              parse_state = PARSE_STATE_s;
              break;
            case 't':
              parse_state = PARSE_STATE_t_1;
              break;
            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        case PARSE_STATE_t_1:

          switch(recv)
            {
            case 's':
              parse_state = PARSE_STATE_s;
              break;
            case 0x0d:
              parse_state = PARSE_STATE_IDLE;

              // Detected START command!!
              return START_COMMAND;

            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        case PARSE_STATE_o:

          switch(recv)
            {
            case 's':
              parse_state = PARSE_STATE_s;
              break;
            case 'p':
              parse_state = PARSE_STATE_p;
              break;
            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        case PARSE_STATE_p:

          switch(recv)
            {
            case 's':
              parse_state = PARSE_STATE_s;
              break;
            case 0x0d:
              parse_state = PARSE_STATE_IDLE;

              // Detected STOP command!!
              return STOP_COMMAND;

            default:
              parse_state = PARSE_STATE_IDLE;
              break;
            }
          break;

        default:
          Serial.println("[BUG]Invalid parse state!!");

          // halt
          while(1)
            {
            }
        }
    }

  return NO_COMMAND;
}
