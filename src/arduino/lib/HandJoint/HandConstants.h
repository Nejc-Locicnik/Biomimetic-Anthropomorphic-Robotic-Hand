
/*###################################  Joint IDs for communication  ###################################*/

#define MCP_L_ID        'A'
#define DIP_L_ID        'B'
#define PIP_L_ID        'C'
#define MCPR_L_ID       'D'
#define MCP_R_ID        'E'
#define DIP_R_ID        'F'
#define PIP_R_ID        'G'
#define MCPR_R_ID       'H'
#define MCP_M_ID        'I'
#define DIP_M_ID        'J'
#define PIP_M_ID        'K'
#define MCPR_M_ID       'L'
#define MCP_I_ID        'M'
#define DIP_I_ID        'N'
#define PIP_I_ID        'O'
#define MCPR_I_ID       'P'
#define MCPR_T_ID       'Q'
#define MCP_T_ID        'R'
#define DIP_T_ID        'S'
#define PIP_T_ID        'T'

/*###################################  Servo pins for each joint  ###################################*/

// Arduino 1
#define MCP_L_SERVO    11
#define PIP_L_SERVO    10
#define DIP_L_SERVO    9
#define MCP_R_SERVO    6
#define PIP_R_SERVO    5
#define DIP_R_SERVO    3

// Arduino 2
#define MCPR_L_SERVO   11
#define MCPR_R_SERVO   10
#define MCPR_M_SERVO    9
#define MCPR_I_SERVO    6
#define MCPR_T_SERVO    5

// Arduino 3
#define MCP_M_SERVO    11
#define PIP_M_SERVO    10
#define DIP_M_SERVO    9
#define MCP_I_SERVO    6
#define PIP_I_SERVO    5
#define DIP_I_SERVO    3

// Arduino 4
#define MCP_T_SERVO    11
#define PIP_T_SERVO    10
#define DIP_T_SERVO    9

/*###################################  Potentiometer pins for each joint  ###################################*/

// Arduino 1
#define MCP_L_POTENTIOMETER    1
#define PIP_L_POTENTIOMETER    2
#define DIP_L_POTENTIOMETER    3
#define MCP_R_POTENTIOMETER    4
#define PIP_R_POTENTIOMETER    5
#define DIP_R_POTENTIOMETER    6

// Arduino 2
#define MCPR_L_POTENTIOMETER   1
#define MCPR_R_POTENTIOMETER   2
#define MCPR_M_POTENTIOMETER   3
#define MCPR_I_POTENTIOMETER   4
#define MCPR_T_POTENTIOMETER   5

// Arduino 3
#define MCP_M_POTENTIOMETER    1
#define PIP_M_POTENTIOMETER    2
#define DIP_M_POTENTIOMETER    3
#define MCP_I_POTENTIOMETER    4
#define PIP_I_POTENTIOMETER    5
#define DIP_I_POTENTIOMETER    6

// Arduino 4
#define MCP_T_POTENTIOMETER    1
#define PIP_T_POTENTIOMETER    2
#define DIP_T_POTENTIOMETER    3

/*###################################  Other constants  ###################################*/

#define PID_ACCURACY        3           // in potentiometer analog read val --> 10 ~= +-3°
#define SERVO_SPEED_CAP     0.95        // in %
#define KP                  3           // proportional part of PID
#define KI                  0.0002       // integral part of PID
#define KD                  0.1         // derivative part of PID