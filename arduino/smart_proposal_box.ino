#include <Servo.h>

#include <NeoSWSerial.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>

//BSP (board configuration)
#define LOCKER_PIN 11
#define BUZZER_PIN 10
#define LED_NO_PIN 9
#define LED_YES_PIN 8
#define NO_BUTTON_PIN 5
#define YES_BUTTON_PIN 6
//proposal geofance info (localization and geofence radius)
#define PROPOSAL_GEOFENCE_LAT 53.93165
#define PROPOSAL_GEOFENCE_LON 14.44584
#define PROPOSAL_GEOFENCE_RADIUS_KM 0.5
//uart cmd sending to RPI
#define CURRENT_LOCATION_CMD ("{\"cmd\": \"current_location\", \"lat\": \"%s\", \"lon\": \"%s\"}")
#define GEOFENCE_LOCATION_CMD ("{\"cmd\": \"geofence_location\", \"lat\": \"%s\", \"lon\": \"%s\"}")
#define PROPOSAL_ACCEPTED_CMD ("{\"cmd\": \"proposal_accepted\", \"lat\": \"%s\", \"lon\": \"%s\"}")
#define PROPOSAL_REJECTED_CMD ("{\"cmd\": \"proposal_rejected\"}")

class Locker: private Servo
{
  private:
    static constexpr int SERVO_DETACH_TIMEOUT_MS = 3000;
   	int signal_pin;
    int unlock_timeout;
    int servo_detach_timeout;
    bool in_progress;
  public:           
    Locker(int pin):signal_pin{ pin }{ 
    }
  	void lock(void);
  	void unlock(void);
  	void unlock(int timeout_ms);
  	void poll_1ms(void);
    bool isInProgress(void);
};

class Geofence
{
  private:
  	struct ReferencePosition
    {
      long double lat;
      long double lon;
    }reference_position;
  	long double fence_radius;
  	static long double to_radians(const long double degree);
  	static long double calculate_distance(long double lat1, long double long1,
                     long double lat2, long double long2);
  public:           
  Geofence(long double lat, long double lon, long double radius):reference_position{lat, lon}, fence_radius{radius}{ 
    }
    int verify_position(long double lat, long double lon, long double& distance);
};

class Indicator 
{
  private:
    uint8_t led_no_pin;
    uint8_t led_yes_pin;
    uint8_t buzzer_pin;
    enum class State
    {
      IDLE = 0,
      BUZZER_AND_YES_LED,
      WAIT,
      NO_LED
    };
    struct 
    {
      State actual_state;
      State next_state;
      uint32_t timeout;
    }fsm;
  public:
    Indicator(uint8_t no_pin, uint8_t yes_pin, uint8_t buzzer_pin):led_no_pin{no_pin}, led_yes_pin{yes_pin}, buzzer_pin{buzzer_pin}{ 
    }
    void poll_1ms(void);
    void enable(void)
    {
      //off
      digitalWrite(led_no_pin, LOW);
      digitalWrite(led_yes_pin, LOW);
      digitalWrite(buzzer_pin, HIGH);
      //clear actual state machine
      memset(&fsm, 0, sizeof(fsm));
      //trig sequence
      fsm.actual_state = Indicator::State::BUZZER_AND_YES_LED;
    };
    void disable(void)
    {
      memset(&fsm, 0, sizeof(fsm));
      //off
      digitalWrite(led_no_pin, LOW);
      digitalWrite(led_yes_pin, LOW);
      digitalWrite(buzzer_pin, HIGH);
    };
};

template<int buff_size> class SimpleCLI
{
  private:
    char buffer[buff_size] = {};
    int buff_idx = 0;
    bool fl_available = false;
    char cmd[buff_size] = {};
    size_t cmd_len = 0;
  public:
    SimpleCLI() {};
    void NewByte(char c)
    {
      if(c != '\n') 
      {
          buffer[buff_idx] = c;   //received character is not a Newline character; save it
          buff_idx++;
      }
      else
      {
          buffer[buff_idx] = '\0';  //insert null character at the end of the received string
          cmd_len = buff_idx+1;
          memcpy(cmd, buffer, cmd_len);
          buff_idx = 0;
          fl_available = true;
      }
      if(buff_idx >= sizeof(buffer))
      {
        buff_idx = 0;
      }
    }

    bool IsAvailable()
    {
      return fl_available;
    }

    uint8_t GetCmd(char* buff, size_t size)
    {
      size_t len = cmd_len;

      if((buff == nullptr) || (len > size))
      {
        return -1;
      }

      memcpy(buff, cmd, cmd_len);

      fl_available = false;

      return 0;
    }
};

struct CmdHandler
{
  char cmd[32];
  void (*handler)();
};

template<int number_of_cmd> class CmdParser
{
  private:
    const struct CmdHandler (&cmd_handlers)[number_of_cmd];
  public:
    CmdParser(const CmdHandler (&handlers)[number_of_cmd]):cmd_handlers{handlers}
    {

    }  
    void Execute(const char* cmd)
    {
      for(auto handler : cmd_handlers)
      {
        if(strcmp(handler.cmd, cmd) == 0)
        {
          handler.handler();
          break;
        }
      }
    }
};


void Locker::unlock(void)
{
  servo_detach_timeout = 0;
  attach(signal_pin);
 	write(90); 
  unlock_timeout = 0;
  servo_detach_timeout = SERVO_DETACH_TIMEOUT_MS;
}

void Locker::unlock(int timeout)
{
  servo_detach_timeout = 0;
  unlock_timeout = timeout;
 	attach(signal_pin);
 	write(90);
  if (unlock_timeout > SERVO_DETACH_TIMEOUT_MS)
  {
    servo_detach_timeout = SERVO_DETACH_TIMEOUT_MS;
  }
  in_progress = true;
}


void Locker::lock(void)
{
  servo_detach_timeout = 0;
  attach(signal_pin);
 	write(180);
  servo_detach_timeout = SERVO_DETACH_TIMEOUT_MS; 
  in_progress = true;
}

bool Locker::isInProgress(void)
{
  return in_progress;
}

void Locker::poll_1ms(void)
{
  	if(servo_detach_timeout)
    {
      servo_detach_timeout--;
      if(servo_detach_timeout == 0)
      {
        detach();
        in_progress = false;
      }
    }
  
  	if(unlock_timeout)
    {
      unlock_timeout--;
      if(unlock_timeout == 0)
      {
        lock();
      }
    }
}

long double Geofence::to_radians(const long double degree)
{
    // cmath library in C++
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

long double Geofence::calculate_distance(long double lat1, long double long1,
                     long double lat2, long double long2)
{
    // Convert the latitudes
    // and longitudes
    // from degree to radians.
    lat1 = to_radians(lat1);
    long1 = to_radians(long1);
    lat2 = to_radians(lat2);
    long2 = to_radians(long2);
     
    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;
 
    long double ans = pow(sin(dlat / 2), 2) +
                          cos(lat1) * cos(lat2) *
                          pow(sin(dlong / 2), 2);
 
    ans = 2 * asin(sqrt(ans));
 
    // Radius of Earth in
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    long double R = 6371;
     
    // Calculate the result
    ans = ans * R;
 
    return ans;
}

int Geofence::verify_position(long double lat, long double lon, long double& distance)
{
  long double distance_km = 0;
  distance_km = calculate_distance(lat, lon, this->reference_position.lat, this->reference_position.lon);
  distance = distance_km;
  if(distance_km > this->fence_radius)
  {
    return 1;
  }
  
  return 0;
}

void Indicator::poll_1ms(void)
{
    switch(fsm.actual_state)
    {
      case Indicator::State::BUZZER_AND_YES_LED:
      {
        digitalWrite(led_yes_pin, HIGH);
        digitalWrite(buzzer_pin, LOW);
        //off
        digitalWrite(led_no_pin, LOW);
        fsm.timeout = 500;
        fsm.actual_state = Indicator::State::WAIT;
        fsm.next_state = Indicator::State::NO_LED;
        break;
      }
      case Indicator::State::WAIT:
      {
        if(fsm.timeout)
        {
          fsm.timeout--;
          if(fsm.timeout == 0)
          {
            fsm.actual_state = fsm.next_state;
          }
        }
        break;
      }
      case Indicator::State::NO_LED:
      {
        digitalWrite(led_no_pin, HIGH);
        //off
        digitalWrite(led_yes_pin, LOW);
        digitalWrite(buzzer_pin, HIGH);
        fsm.timeout = 500;
        fsm.actual_state = Indicator::State::WAIT;
        fsm.next_state = Indicator::State::BUZZER_AND_YES_LED;
        break;
      }
    }
}


// Main 
static unsigned long actual_time;
static unsigned long prev_time;
static NMEAGPS  gps;
static gps_fix  fix;
static long double last_lat = 0;
static long double last_lon = 0;
static Locker locker(LOCKER_PIN);
static Geofence geofence(PROPOSAL_GEOFENCE_LAT, PROPOSAL_GEOFENCE_LON, PROPOSAL_GEOFENCE_RADIUS_KM);
static Indicator indicator(LED_NO_PIN, LED_YES_PIN, BUZZER_PIN);
static SimpleCLI<32> simple_cli;


static void ServiceLockCmd();
static void ServiceUnlockCmd();
static void RequestProposalCmd(void);

const struct CmdHandler cmd_handlers[] = 
{
  {"lock", ServiceLockCmd},
  {"unlock", ServiceUnlockCmd},
  {"proposal_request", RequestProposalCmd}
};

static CmdParser<sizeof(cmd_handlers)/sizeof(cmd_handlers[0])> cmd_parser(cmd_handlers);

enum ProposalState
{
  PROPOSAL_IDLE = 0,
  PROPOSAL_INIT,
  PROPOSAL_UNLOCK,
  PROPOSAL_INIT_REQUEST,
  PROPOSAL_REQUEST,
  PROPOSAL_ACCPETED,
  PROPOSAL_REJECTED
}proposal_state = PROPOSAL_IDLE;

// CheckPosition
static void CheckPosition(void)
{
  if (fix.valid.location) {
    long double distance;
    long double lat, lon;
    lat = fix.latitude();
    lon = fix.longitude();
    if(geofence.verify_position(lat, lon, distance) == 0)
    {
      char cmd[200] = {0};
      String lat_str = String(static_cast<float>(lat), 6);
      String lon_str = String(static_cast<float>(lon), 6);
      //send command
      sprintf(cmd, CURRENT_LOCATION_CMD, lat_str.c_str(), lon_str.c_str());
      Serial.println(cmd);
    }
    else
    {
      char cmd[200] = {0};
      String lat_str = String(static_cast<float>(PROPOSAL_GEOFENCE_LAT), 6);
      String lon_str = String(static_cast<float>(PROPOSAL_GEOFENCE_LON), 6);
      //send command
      sprintf(cmd, GEOFENCE_LOCATION_CMD, lat_str.c_str(), lon_str.c_str());
      Serial.println(cmd);
    }
    last_lat = lat;
    last_lon = lon; 
  }   
} 

// This is the main GPS parsing loop.
static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    CheckPosition();
  }
}

// This is the main Cmd parsing loop.
static void CmdLoop()
{
  if(Serial.available())
  {
    char byte = Serial.read();
    simple_cli.NewByte(byte);
    if(simple_cli.IsAvailable())
    {
      char cmd[32] = {};
      simple_cli.GetCmd(cmd, sizeof(cmd));
      cmd_parser.Execute(cmd);
    }
  }
}

static void ProposalFSM(void)
{
  int no_state = 0;
  int yes_state = 0;
  static uint8_t send_cnt = 0;

  switch(proposal_state)
  {
    case PROPOSAL_IDLE:
    {
      break;
    }
    case PROPOSAL_INIT:
    {
      locker.unlock();
      proposal_state = PROPOSAL_UNLOCK;
      break;
    }
    case PROPOSAL_UNLOCK:
    {
      if(locker.isInProgress() == false)
      {
        //done
        proposal_state = PROPOSAL_INIT_REQUEST;
      }
      break;
    }
    case PROPOSAL_INIT_REQUEST:
    {
      indicator.enable();
      proposal_state = PROPOSAL_REQUEST;
      break;
    }
    case PROPOSAL_REQUEST:
    {
      yes_state = digitalRead(YES_BUTTON_PIN);
      no_state = digitalRead(NO_BUTTON_PIN);
      if (yes_state == LOW)
      {
        proposal_state = PROPOSAL_ACCPETED;
      } else if (no_state == LOW)
      {
        proposal_state = PROPOSAL_REJECTED;
      }
      break;
    }
    case PROPOSAL_ACCPETED:
    {
      char cmd[200] = {0};
      String lat_str = String(static_cast<float>(last_lat), 6);
      String lon_str = String(static_cast<float>(last_lon), 6);
      //send command
      sprintf(cmd, PROPOSAL_ACCEPTED_CMD, lat_str.c_str(), lon_str.c_str());
      Serial.println(cmd);
      indicator.disable();
      send_cnt++;
      if(send_cnt >= 10)
      {
        proposal_state = PROPOSAL_IDLE;
      }
      break;
    }
    case PROPOSAL_REJECTED:
    {
      indicator.disable();
      Serial.println(PROPOSAL_REJECTED_CMD);
      send_cnt++;
      if(send_cnt >= 10)
      {
        proposal_state = PROPOSAL_IDLE;
      }
      break;
    }
    default:
    {
      break;
    }
  }
}

static void ServiceLockCmd()
{
  locker.lock();
}

static void ServiceUnlockCmd()
{
  locker.unlock();
}

static void RequestProposalCmd(void)
{
  proposal_state = PROPOSAL_INIT;
}

void setup()
{
  DEBUG_PORT.begin(9600);
  gpsPort.begin( 9600 );
  pinMode(YES_BUTTON_PIN, INPUT);
  pinMode(NO_BUTTON_PIN, INPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_YES_PIN, OUTPUT);
  pinMode(LED_NO_PIN, OUTPUT);
}

void loop()
{
  actual_time = millis();
  if(actual_time > prev_time)
  {
    // 1ms tick
    // module tasks
    locker.poll_1ms();
    indicator.poll_1ms();
    prev_time = actual_time;
  }
  // module tasks
  GPSloop();
  CmdLoop();
  // application task
  ProposalFSM();

}