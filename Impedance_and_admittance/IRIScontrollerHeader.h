


#define E_STOP 0
#define TORQUE_CONTROL 1
#define SPEED_CONTROL 2

#define VEL_FILTERING_STUFF 5
#define ACC_FILTERING_STUFF 5
#define INT_DT 0.001
#define GEAR_RATIO 20
#define GRAVITY 9.8

volatile float m_max_damping = 70.0;
volatile float m_min_damping = 30.0;
volatile float p_max_damping = 70.0;
volatile float p_min_damping = 30.0;
volatile float m_limit_speed = 0.3;
volatile float p_limit_speed = 0.3;
volatile float m_mid_limit_speed = 0.1;
volatile float p_mid_limit_speed = 0.1;

typedef struct CONTROL_STATE{
	int state_;
}C_state;

// Queue Variable
typedef struct {
    int max;
    int num;
    int rear;
    int front;
    float *que;
}Queue;

typedef struct Impednace_Controller_Structure{
	float Im_velo;
	float Im_position;
	float Im_acc;
	float Im_force;
	float Im_Fd;
	float Im_Ref_A;
	float Im_x_dot;
	float Im_x_dot0;
	float Im_damping;
	float Im_Friction;
}Im_Structure;

typedef struct Admittance_Controller_Structure{
	float Ad_velo;
	float Ad_x_dot;
	float Ad_x2dot;
	float Ad_damping;
	float Ad_SpeedRef_krpm;
}Ad_Structure;

typedef struct Adaptive_Windowing_Filter{
	float sum_a;
	float sum_b;
	float pos_L;
	float vel_L;
	bool filter_off;
	int n;
	float bn;
	float d;
	float Dt;
	int num_buffer;
}AWFilter;


// Queue Function
int Initialize(Queue *q, int max) {
    q->num = q->front = q->rear = 0;
    if ((q->que = (float*)calloc(max, sizeof(float))) == NULL) {
        q->max = 0;
        return -1;
    }
    q->max = max;
    return 0;
}

int Deque(Queue *q) {
    if (q->num <= 0){
        return -1;
    }
    else {
        q->num--;
        q->front++;
        if (q->front == q->max)
            q->front = 0;
        return 0;
    }
}

int Enque(Queue *q, float x) {
    if (q->num >= q->max){
        Deque(q);
    }

    q->num++;
    q->que[q->rear++] = x;
    if (q->rear == q->max)
        q->rear = 0;
    return 0;
}

float out_value(Queue *q, int i){
    int k = q->rear - i - 1;

    if (k < 0){
        k = q->max + k;
    }
    if (q->num <= i){
        return 0;
    }
    else{
        return q->que[k];
    }
}
float back(Queue *q) {
    if (q->num <= 0)
        return -1;
    else
        return q->que[q->rear];
}

float front(Queue *q) {
    if (q->num <= 0)
        return -1;
    else
        return q->que[q->front];
}

/*--- End of Queue Function ---*/
void VariableInitialize(Im_Structure *Im_main, Ad_Structure *Ad_main, AWFilter *Filt_velo, AWFilter *Filt_acc, C_state *st, Queue *q_pos, Queue *q_vel)
{
    Im_main->Im_velo = 0.0;
    Im_main->Im_position = 0.0;
    Im_main->Im_acc = 0.0;
    Im_main->Im_force = 0.0;
    Im_main->Im_Fd = 0.0;
    Im_main->Im_Ref_A = 0.0;
    Im_main->Im_x_dot = 0.0;
    Im_main->Im_x_dot0 = 0.0;
    Im_main->Im_damping = 0.1;
    Im_main->Im_Friction = 20.0;

    Ad_main->Ad_velo = 0.0;
    Ad_main->Ad_x_dot = 0.0;
    Ad_main->Ad_x2dot = 0.0;
    Ad_main->Ad_damping = 30.0;
    Ad_main->Ad_SpeedRef_krpm = 0.0;

    Filt_velo->sum_a = 0.0;
    Filt_velo->sum_b = 0.0;
    Filt_velo->pos_L = 0.0;
    Filt_velo->vel_L = 0.0;
    Filt_velo->filter_off = false;
    Filt_velo->n = 0;
    Filt_velo->bn = 0.0;
    Filt_velo->d = 0.00016;
    Filt_velo->Dt = INT_DT;
    Filt_velo->num_buffer = VEL_FILTERING_STUFF;

    Filt_acc->sum_a = 0.0;
    Filt_acc->sum_b = 0.0;
    Filt_acc->pos_L = 0.0;
    Filt_acc->vel_L = 0.0;
    Filt_acc->filter_off = false;
    Filt_acc->n = 0;
    Filt_acc->bn = 0.0;
    Filt_acc->d = 0.0016;
    Filt_acc->Dt = INT_DT;
    Filt_acc->num_buffer = ACC_FILTERING_STUFF;

    //Queue Intialize
    Initialize(q_pos, Filt_velo->num_buffer);
    Initialize(q_vel, Filt_acc->num_buffer);
}


// Impedance Controller
void Impedance_controller(Im_Structure *Im_main, AWFilter *Filt_velo, Queue *q_pos, AWFilter *Filt_acc, Queue *q_vel, float Mass_, C_state *st)
{
    // Torque Control
    //st->state_ = TORQUE_CONTROL;
    if(st->state_ == SPEED_CONTROL)st->state_ = TORQUE_CONTROL;

    // 1/pi/0.1/1000*60 m/s >> krpm
    // pi*0.1*1000/60 krpm >> m/s
    //Im_main->Im_velo = _IQtoF(gMotorVars.SpeedQEP_krpm)*5.235987;
    Im_main->Im_position += Im_main->Im_velo  * INT_DT; // INT_DT : sampling time -> 0.001 s

    // Adative Windowing Filter Variable Initialize
    // velocity filtering
    int i = 0;
    Filt_velo->n = 0;
    Filt_velo->bn = 0.0;
    Filt_velo->pos_L = 0.0;
    Enque(q_pos,Im_main->Im_position);
    Filt_velo->filter_off = false;

    //Adaptive Windowing Filter -- velocity
    if(q_pos->num >= FILTERING_STUFF){
      while((Filt_velo->filter_off == false) && (Filt_velo->n < Filt_velo->num_buffer)){
          Filt_velo->n = Filt_velo->n + 1;
          Filt_velo->sum_a = 0.0;
          Filt_velo->sum_b = 0.0;

          for(i=0;i<Filt_velo->n;i++){
              Filt_velo->sum_a = Filt_velo->sum_a + out_value(q_pos,i);
              Filt_velo->sum_b = Filt_velo->sum_b + i * out_value(q_pos,i);
          }
          Filt_velo->bn = (Filt_velo->n * Filt_velo->sum_a - 2 * Filt_velo->sum_b)/(Filt_velo->n*(Filt_velo->n+1)*(Filt_velo->n+2)/6.0);
          i = 0;
          for(i=1;i<Filt_velo->n-1;i++){
              Filt_velo->pos_L = back(q_pos) - Filt_velo->bn*i;
              //Enque(&q_pos_L,pos_L);
              if(fabs(out_value(q_pos,i) - Filt_velo->pos_L) > Filt_velo->d){
                  Filt_velo->filter_off = true;
              }
          }
      }

      Im_main->Im_x_dot = (back(q_pos) - out_value(q_pos,Filt_velo->n - 1))/((Filt_velo->n - 1) * Filt_velo->Dt);
    }else{
      Im_main->Im_x_dot = Im_main->Im_velo;
    }

    // Adative Windowing Filter Variable Initialize
    // accerlation filtering
    int j = 0;
    Filt_acc->n = 0;
    Filt_acc->bn = 0.0;
    Filt_acc->vel_L = 0.0;
    Enque(q_vel,Im_main->Im_x_dot);
    Filt_acc->filter_off = false;
    if(q_vel->num >= FILTERING_STUFF){
        while((Filt_acc->filter_off == false) && (Filt_acc->n < Filt_acc->num_buffer)){
            Filt_acc->n= Filt_acc->n + 1;
            Filt_acc->sum_a = 0.0;
            Filt_acc->sum_b = 0.0;

            for(j=0;j<Filt_acc->n;j++){
                Filt_acc->sum_a = Filt_acc->sum_a + out_value(q_vel,j);
                Filt_acc->sum_b = Filt_acc->sum_b + j * out_value(q_vel,j);
            }
            Filt_acc->bn = (Filt_acc->n * Filt_acc->sum_a - 2 * Filt_acc->sum_b)/(Filt_acc->n * (Filt_acc->n+1) * (Filt_acc->n+2)/6.0);
            j = 0;
            for(j=1;j<Filt_acc->n-1;j++){
                Filt_acc->vel_L = back(q_vel) - Filt_acc->bn*j;
                if(fabs(out_value(q_vel,j) - Filt_acc->vel_L) > Filt_acc->d){
                    Filt_acc->filter_off = true;
                }
            }
        }
        Im_main->Im_acc = (back(q_vel) - out_value(q_vel, Filt_acc->n-1))/((Filt_acc->n-1) * Filt_acc->Dt);
    }
    else{
        Im_main->Im_acc = (Im_main->Im_x_dot - Im_main->Im_x_dot0) / INT_DT;
    }

    // Impedance Control
    Im_main->Im_Fd = Mass_ * Im_main->Im_acc + Im_main->Im_damping * Im_main->Im_x_dot + Mass_ * GRAVITY - Im_main->Im_Friction/(Mass_ * GRAVITY);

    // Ouput force
    Im_main->Im_force = (Im_main->Im_Fd)/GRAVITY;

    // Modified force -> Torque Controller
    Im_main->Im_Ref_A = -(0.002739506032886 * Im_main->Im_force * Im_main->Im_force + 0.660059366830369 * Im_main->Im_force + 0.323310384925088);
    Im_main->Im_x_dot0 = Im_main->Im_x_dot;

    return;
}

void Admittance_controller(Ad_Structure *Ad_main, float Mass_, float LoadCell, C_state *st)
{

    if(st->state_ == TORQUE_CONTROL)st->state_ = SPEED_CONTROL;

    //Ad_main->Ad_velo = _IQtoF(gMotorVars.SpeedQEP_krpm)*5.235987;
    Ad_main->Ad_x2dot = (LoadCell * GRAVITY - Mass_ * GRAVITY - Ad_main->Ad_damping * Ad_main->Ad_x_dot) / Mass_;

    // Damping change using direction
    if(Ad_main->Ad_velo < -m_limit_speed){
        Ad_main->Ad_damping = m_min_damping;
    }else if(Ad_main->Ad_velo >= -m_limit_speed && Ad_main->Ad_velo < -m_mid_limit_speed){
        Ad_main->Ad_damping = (m_max_damping - m_min_damping)/(m_limit_speed - m_mid_limit_speed)*x_dot + m_max_damping + (m_max_damping-m_min_damping)/(m_limit_speed - m_mid_limit_speed)*m_mid_limit_speed;
    }else if(Ad_main->Ad_velo>= -m_mid_limit_speed && Ad_main->Ad_velo <= p_mid_limit_speed){
        Ad_main->Ad_damping = m_max_damping;
    }else if(Ad_main->Ad_velo > p_mid_limit_speed && Ad_main->Ad_velo <= p_limit_speed ){
        Ad_main->Ad_damping = (p_min_damping - m_max_damping)/(p_limit_speed-p_mid_limit_speed)*x_dot + m_max_damping - (p_min_damping-m_max_damping)/(p_limit_speed - p_mid_limit_speed)*p_mid_limit_speed;
    }else if(Ad_main->Ad_velo > p_limit_speed){
        Ad_main->Ad_damping  = p_min_damping;
    }

    if(st->state_!=SPEED_CONTROL) Ad_main->Ad_x_dot = 0.0;
    Ad_main->Ad_x_dot += Ad_main->Ad_x2dot * INT_DT;    // m/s^2 >> m/s

    // Speed Controller (Desired Speed)
    Ad_main->Ad_SpeedRef_krpm = Ad_main->Ad_x_dot * 0.1909859; // 1/pi/0.1/1000*60*GEAR_RATIO m/s >> krpm

    return;
}
