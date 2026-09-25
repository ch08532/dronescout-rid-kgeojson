// DroneScout Remote ID traffic simulator.
//
// Transmitter -> air (25-byte ASTM F3411 messages) -> receiver ODID_UAS_Data,
// using the real opendroneid-core-c library at the commit the DroneScout
// firmware uses (4785de45...). Output: one JSON line per drone per second with
// ground truth and the receiver-side ODID_UAS_Data bytes (hex) = sensor UASdata.
//
// Default scenario: 4 drones around Oslo Airport Gardermoen (ENGM), all below
// 120 m (394 ft) AGL. Every pattern returns to its start at the end of the
// scenario, so the publisher can loop it seamlessly.
//
//   ./odid_sim [--lat 60.1939] [--lon 11.1004] [--ground-msl 208]
//              [--geoid 38.3] [--seconds 600] > sim.jsonl  2> layout.txt
#define _USE_MATH_DEFINES          /* M_PI on MSVC */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include <getopt.h>          /* Linux, macOS, MinGW/MSYS2 (not MSVC) */
#include "opendroneid.h"

/* catches a compiler/ABI that lays the struct out differently from the sensor */
_Static_assert(sizeof(ODID_UAS_Data) == 920, "ODID_UAS_Data must be 920 bytes to match DroneScout UASdata");

#define OFF(f) fprintf(stderr, "%-34s %4zu\n", #f, offsetof(ODID_UAS_Data, f))
#define MAX_AGL_M 120.0          /* stay below 400 ft (121.9 m) AGL */
#define D2R (M_PI / 180.0)

static double c_lat = 60.1939, c_lon = 11.1004;   // ENGM ARP (approx.)
static double ground_msl = 208.0;                  // ENGM elevation ~681 ft MSL
static double geoid_n = 38.3;                      // EGM96 at ENGM (HAE - MSL), m
static int    seconds = 600;

typedef struct { double e, n, agl, course, vh, vv; } State;   // ENU metres rel. to ARP

static void hex(const void *p, size_t n){const unsigned char*b=p;for(size_t i=0;i<n;i++)printf("%02x",b[i]);}
static double wrap360(double a){ a=fmod(a,360.0); return a<0?a+360.0:a; }
static double clampagl(double h){ return h<0?0:(h>MAX_AGL_M?MAX_AGL_M:h); }
// periodic triangle wave 0..1..0 over period P
static double tri(double t,double P){ double x=fmod(t,P)/P; return x<0.5?2*x:2-2*x; }

// 1: orbit centred 1.2 km east of the ARP (near the eastern runway), r=250 m, 100 m AGL, ~8 m/s
static State orbit(double t){
  double r=250, v=8, w=v/r;
  // choose an integer number of laps in the scenario so the loop is seamless
  double laps=fmax(1,round(w*seconds/(2*M_PI))); w=laps*2*M_PI/seconds; v=w*r;
  double a=w*t; State s={1200+r*cos(a), r*sin(a), 100, 0, v, 0};
  s.course=wrap360(90-(a*180/M_PI+90));             // tangent, counter-clockwise
  return s;
}
// 2: transit 3.5 km north of the ARP, west<->east 6 km legs at 110 m AGL (~15-20 m/s)
static State transit(double t){
  double L=6000, legs=fmax(2,2*round(15.0*seconds/(2*L)));
  double P=2.0*seconds/legs, x=tri(t,P), v=2*L/P;
  int eastbound=fmod(t,P)<P/2;
  State s={-L/2+L*x, 3500, 110, eastbound?90:270, v, 0}; return s;
}
// 3: vertical profile 2 km south-west: climb 3 m/s to 115 m, hover, descend, repeat
static State vertical(double t){
  double P=seconds/2.0, x=fmod(t,P), climb=115/3.0, hover=P-2*climb, h, vv;
  if(x<climb){ h=3*x; vv=3; } else if(x<climb+hover){ h=115; vv=0; } else { h=115-3*(x-climb-hover); vv=-3; }
  State s={-1400,-1400,clampagl(h),0,0,vv}; return s;
}
// 4: survey (lawnmower) 400 x 300 m box, 2.5 km south, 60 m AGL (~6-10 m/s)
static State survey(double t){
  const double W=400, H=300, lane=50; int lanes=(int)(H/lane)+1;
  double path=lanes*W+(lanes-1)*lane, P=2.0*seconds/ fmax(2,2*round(6.0*seconds/(2*path)));
  double d=tri(t,P)*path, v=2*path/P; int back=fmod(t,P)>=P/2;
  int k=0; double e=0,n=0,course=90;
  for(k=0;k<lanes;k++){
    if(d<=W){ e=(k%2==0)?d:W-d; n=k*lane; course=(k%2==0)?90:270; break; } d-=W;
    if(k<lanes-1 && d<=lane){ e=(k%2==0)?W:0; n=k*lane+d; course=0; break; } d-=lane;
  }
  if(back) course=wrap360(course+180);
  State s={-W/2+e, -2500-H/2+n, 60, course, v, 0}; return s;
}

typedef struct { const char *uasid, *mac, *link, *mfr, *model, *opid, *selfid;
                 ODID_uatype_t uatype; State (*fn)(double); double op_e, op_n; int bt_legacy; } Drone;
static Drone drones[] = {
  {"SIM0OSL0000000000001","60:60:1F:00:00:01","BLE legacy","DJI","Mavic 3 Enterprise","NOR-SIM-0001","Orbit",
   ODID_UATYPE_HELICOPTER_OR_MULTIROTOR, orbit, 1200,-400, 1},
  {"SIM0OSL0000000000002","60:60:1F:00:00:02","WiFi beacon","Autel","EVO II Pro","NOR-SIM-0002","Transit",
   ODID_UATYPE_HELICOPTER_OR_MULTIROTOR, transit, -3200,3300, 0},
  {"SIM0OSL0000000000003","60:60:1F:00:00:03","BLE long range","Skydio","X10","NOR-SIM-0003","Climb and hover",
   ODID_UATYPE_HELICOPTER_OR_MULTIROTOR, vertical, -1420,-1420, 0},
  {"SIM0OSL0000000000004","60:60:1F:00:00:04","WiFi NaN","Wingtra","WingtraOne","NOR-SIM-0004","Survey",
   ODID_UATYPE_HYBRID_LIFT, survey, -300,-2300, 0},
};
#define ND (sizeof drones/sizeof drones[0])

static void enu2ll(double e,double n,double *lat,double *lon){
  *lat=c_lat+n/111320.0; *lon=c_lon+e/(111320.0*cos(c_lat*D2R));
}

int main(int argc,char**argv){
  static struct option o[]={{"lat",1,0,'a'},{"lon",1,0,'o'},{"ground-msl",1,0,'g'},{"geoid",1,0,'n'},{"seconds",1,0,'s'},{0,0,0,0}};
  int c; while((c=getopt_long(argc,argv,"",o,0))!=-1){
    switch(c){case 'a':c_lat=atof(optarg);break;case 'o':c_lon=atof(optarg);break;case 'g':ground_msl=atof(optarg);break;
              case 'n':geoid_n=atof(optarg);break;case 's':seconds=atoi(optarg);break;default:return 2;}}
  if(seconds<60){fprintf(stderr,"--seconds must be >= 60\n");return 2;}

  fprintf(stderr,"sizeof(ODID_UAS_Data) = %zu\n", sizeof(ODID_UAS_Data));
  OFF(BasicID[0].UAType);OFF(BasicID[0].IDType);OFF(BasicID[0].UASID);OFF(BasicID[1]);
  OFF(Location.Status);OFF(Location.Direction);OFF(Location.SpeedHorizontal);OFF(Location.SpeedVertical);
  OFF(Location.Latitude);OFF(Location.Longitude);OFF(Location.AltitudeBaro);OFF(Location.AltitudeGeo);
  OFF(Location.HeightType);OFF(Location.Height);OFF(Location.HorizAccuracy);OFF(Location.VertAccuracy);
  OFF(Location.BaroAccuracy);OFF(Location.SpeedAccuracy);OFF(Location.TSAccuracy);OFF(Location.TimeStamp);
  OFF(Auth[0]);OFF(SelfID.DescType);OFF(SelfID.Desc);
  OFF(System.OperatorLocationType);OFF(System.ClassificationType);OFF(System.OperatorLatitude);
  OFF(System.OperatorLongitude);OFF(System.OperatorAltitudeGeo);OFF(System.Timestamp);
  OFF(OperatorID.OperatorIdType);OFF(OperatorID.OperatorId);
  OFF(BasicIDValid);OFF(LocationValid);OFF(AuthValid);OFF(SelfIDValid);OFF(SystemValid);OFF(OperatorIDValid);

  ODID_UAS_Data rx[ND]; for(size_t d=0;d<ND;d++) odid_initUasData(&rx[d]);   // receiver state per drone
  const long long rx0 = 1790000000123LL;   // nominal time base; the publisher restamps to wall-clock time

  for(int t=0;t<seconds;t++){
    for(size_t d=0;d<ND;d++){
      Drone *D=&drones[d]; State s=D->fn(t);
      long long rx_ms=rx0+t*1000LL;   /* 64-bit: long is 32-bit on Windows */
      double lat,lon; enu2ll(s.e,s.n,&lat,&lon);
      double msl=ground_msl+s.agl, hae=msl+geoid_n;

      ODID_BasicID_data bid; odid_initBasicIDData(&bid);
      bid.UAType=D->uatype; bid.IDType=ODID_IDTYPE_SERIAL_NUMBER; strncpy(bid.UASID,D->uasid,ODID_ID_SIZE);
      ODID_Location_data loc; odid_initLocationData(&loc);
      loc.Status = s.agl>0.5 ? ODID_STATUS_AIRBORNE : ODID_STATUS_GROUND;
      loc.Latitude=lat; loc.Longitude=lon;
      loc.AltitudeGeo=(float)hae; loc.AltitudeBaro=(float)msl;          // baro ~ MSL (standard atmosphere)
      loc.HeightType=ODID_HEIGHT_REF_OVER_GROUND; loc.Height=(float)s.agl;
      loc.Direction=(float)s.course; loc.SpeedHorizontal=(float)s.vh; loc.SpeedVertical=(float)s.vv;
      loc.HorizAccuracy=ODID_HOR_ACC_3_METER; loc.VertAccuracy=ODID_VER_ACC_10_METER;
      loc.BaroAccuracy=ODID_VER_ACC_3_METER; loc.SpeedAccuracy=ODID_SPEED_ACC_1_METERS_PER_SECOND;
      loc.TSAccuracy=ODID_TIME_ACC_0_2_SECOND;
      loc.TimeStamp=(float)(((rx_ms-300)/100%36000)/10.0);
      ODID_System_data sys; odid_initSystemData(&sys);
      double olat,olon; enu2ll(D->op_e,D->op_n,&olat,&olon);
      sys.OperatorLocationType=ODID_OPERATOR_LOCATION_TYPE_TAKEOFF; sys.ClassificationType=ODID_CLASSIFICATION_TYPE_EU;
      sys.OperatorLatitude=olat; sys.OperatorLongitude=olon; sys.OperatorAltitudeGeo=(float)(ground_msl+geoid_n);
      sys.AreaCount=1; sys.CategoryEU=ODID_CATEGORY_EU_OPEN; sys.ClassEU=ODID_CLASS_EU_CLASS_2;
      sys.Timestamp=(uint32_t)((rx_ms/1000)-1546300800LL);
      ODID_OperatorID_data op; odid_initOperatorIDData(&op);
      op.OperatorIdType=ODID_OPERATOR_ID; strncpy(op.OperatorId,D->opid,ODID_ID_SIZE);
      ODID_SelfID_data sid; odid_initSelfIDData(&sid);
      sid.DescType=ODID_DESC_TYPE_TEXT; strncpy(sid.Desc,D->selfid,ODID_STR_SIZE);

      ODID_BasicID_encoded eb; ODID_Location_encoded el; ODID_System_encoded es; ODID_OperatorID_encoded eo; ODID_SelfID_encoded ei;
      if(encodeBasicIDMessage(&eb,&bid)||encodeLocationMessage(&el,&loc)||encodeSystemMessage(&es,&sys)
         ||encodeOperatorIDMessage(&eo,&op)||encodeSelfIDMessage(&ei,&sid)){fprintf(stderr,"encode failed drone %zu t=%d\n",d,t);return 1;}

      // over the air: BT4 legacy sends one message type per frame (ID/system less often);
      // BT5 long range and Wi-Fi send a full message pack every time.
      decodeOpenDroneID(&rx[d],(uint8_t*)&el);
      if(!D->bt_legacy || t%3==0) decodeOpenDroneID(&rx[d],(uint8_t*)&eb);
      if(!D->bt_legacy || t%3==2){ decodeOpenDroneID(&rx[d],(uint8_t*)&es); decodeOpenDroneID(&rx[d],(uint8_t*)&eo); decodeOpenDroneID(&rx[d],(uint8_t*)&ei);}

      printf("{\"t\":%d,\"drone\":%zu,\"rx_ms\":%lld,\"mac\":\"%s\",\"link\":\"%s\",\"manufacturer\":\"%s\",\"model\":\"%s\","
             "\"truth\":{\"lat\":%.7f,\"lon\":%.7f,\"agl\":%.2f,\"msl\":%.2f,\"hae\":%.2f,\"alt_geo\":%.2f,"
             "\"dir\":%.2f,\"vh\":%.2f,\"vv\":%.2f,\"ts\":%.1f},\"uas_hex\":\"",
             t,d,rx_ms,D->mac,D->link,D->mfr,D->model,lat,lon,s.agl,msl,hae,hae,s.course,s.vh,s.vv,loc.TimeStamp);
      hex(&rx[d],sizeof rx[d]); printf("\"}\n");
    }
  }
  fprintf(stderr,"scenario: %zu drones, %d s, centre %.5f,%.5f, ground %.1f m MSL, geoid N %.2f m, max AGL %.0f m\n",
          ND,seconds,c_lat,c_lon,ground_msl,geoid_n,MAX_AGL_M);
  return 0;
}
