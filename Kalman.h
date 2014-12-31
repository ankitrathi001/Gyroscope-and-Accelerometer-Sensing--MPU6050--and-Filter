
extern float angle_x;
extern float q_bias_x;
extern float rate_x;

extern float angle_y;
extern float q_bias_y;
extern float rate_y;

extern float kalman_updateX(const float q_m,const float ax_m, const float az_m,const float dt);
extern float kalman_updateY(const float q_m,const float ax_m, const float az_m,const float dt);
