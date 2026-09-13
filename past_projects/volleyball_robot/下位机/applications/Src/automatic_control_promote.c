#include "include.h"
#include "automatic_control_promote.h"


float low_pass_filter(float input, float last_output,float alpha)
{
	
	return alpha*input+(1.0f-alpha)*last_output;
}

speed_designer body_acc_design[3]={0};
float body_feedforword[3]={0};
float body_accs[3]={0};

void vel_designer_init(speed_designer*this,float max_acc)
{
	
	for(uint8_t i=0;i<3;++i)
	{
		
		this[i].max_acc=max_acc;
		
	}
	
}

void designer_update(speed_designer* this)
{
	
	float delta_v = this->expected_vel - this->current_vel;//delta表示变化量，不是那个delta机构
    float max_delta = this->max_acc * DT;

    if (fabsf(delta_v) <= max_delta) {
        this->current_vel = this->expected_vel;
        this->current_acc = 0.0f;
    } else {
        float step = (delta_v > 0.0f) ? max_delta : -max_delta;
        this->current_vel += step;
        this->current_acc = step / DT;
    }
	
}

void feedforword_control(speed_designer*this,float *exp_vel)
{
	for(uint8_t i=0;i<3;++i)
	{
		this[i].expected_vel=exp_vel[i];
		designer_update(&this[i]);
		body_feedforword[i]=this[i].current_acc;
	}
	
	matrix_multiply(inverse_solution_matrix,body_feedforword,body_accs);
	
}

void motion_profiles_reset(void)
{
    for (uint8_t i = 0; i < 3; ++i) {
        body_acc_design[i].expected_vel = 0;
        body_acc_design[i].current_vel = 0;
        body_acc_design[i].current_acc = 0;
        body_accs[i] = 0;
        body_feedforword[i] = 0;
    }
}

spatium_designer xyw_spatium_designer[3]={0};

void spatium_designer_init(spatium_designer*this,float v_m,float a_m)
{
	
	this->v_max=v_m;
	this->a_max=a_m;
	
	this->current_s=0;
	this->current_v=0;
	this->current_t=0;
	
}

void xy_spatium_designer_init(spatium_designer*this,float v_m,float a_m)
{
	
	for(uint8_t i=0;i<2;++i)
	{
		
		spatium_designer_init(&this[i],v_m,a_m);
		
	}
	
}

void spatium_designer_set_target(spatium_designer*this,float total_spatium)
{
	
	this->total_spatium=fabsf(total_spatium);
	this->finished=False;
	this->current_s=0;
	this->current_v=0;
	this->current_t=0;
	this->towards=total_spatium>0? 1:-1;
	
	this->t[0]=this->v_max/this->a_max;
	this->spatium_acc=0.5f*this->a_max*this->t[0]*this->t[0];
	this->spatium_const=this->total_spatium-2*this->spatium_acc;
	
	if(this->total_spatium>2.0f*this->spatium_acc)//v_m可以达到
	{
		this->t[1]=(this->total_spatium-2.0f*this->spatium_acc)/this->v_max;
	}else{/*v_m不可到达*/
		this->t[0]=sqrt(this->total_spatium/this->a_max);
		this->t[1]=0;//使用三角形规划
	};
}

void spatium_designer_update(spatium_designer*this)
{
	if(this->finished==True) return;//加容错
		if(this->current_t<this->t[0])
		{
			this->current_s+=this->current_v*DT+0.5f*this->a_max*DT*DT;
			this->current_v+=this->a_max*DT;
		}//加速段
		else if(this->current_t<this->t[0]+this->t[1])
		{
			this->current_v=this->v_max;
			this->current_s+=this->v_max*DT;
		}//匀速段
		else
		{
			this->current_s += this->current_v * DT - 0.5f * this->a_max * DT * DT;
			this->current_v-=this->a_max*DT;
			if(this->current_v<0) this->current_v=0;
			if (this->current_v < 0) this->current_v = 0;
		}//减速段
		
		if(this->current_s >= this->total_spatium-0.1f/*给与容差*/ || this->current_t > (2*this->t[0] + this->t[1]))
		{
			this->current_s = this->total_spatium;
			this->current_v = 0;
			this->current_t = 0;
			this->finished=True;
		}
		this->v_out=this->towards>0? this->current_v:-this->current_v;
	this->current_t+=DT;

}





void SPlanner_Init(S_CurvePlanner* planner, float j_max, float a_max, float dt) {
    planner->v_target = 0.0f;
    planner->v_curr   = 0.0f;
    planner->a_curr   = 0.0f;
    planner->j_max    = j_max;
    planner->a_max    = a_max;
    planner->dt       = dt;
}

void SPlanner_SetTarget(S_CurvePlanner* planner, float target) {
    planner->v_target = target;
}

float SPlanner_Update(S_CurvePlanner* planner) {
    float dv = planner->v_target - planner->v_curr;
    float abs_dv = fabsf(dv);  // 速度差的绝对值
    int8_t dir_sign = (dv >= 0.0f) ? 1 : -1;  // 判断目标方向

    // 如果当前加速度方向与目标方向相反，需要先减速
    if (dv * planner->a_curr < 0.0f) {
        planner->a_curr -= dir_sign * planner->j_max * planner->dt;
    } else {
        planner->a_curr += dir_sign * planner->j_max * planner->dt;
    }

    // 限制加速度范围
    float a_limit = planner->a_max;
    if (planner->a_curr >  a_limit) planner->a_curr =  a_limit;
    if (planner->a_curr < -a_limit) planner->a_curr = -a_limit;

    // 更新速度
    planner->v_curr += planner->a_curr * planner->dt;

    // 防止超调：如果已经超过目标速度，则强行设定为目标速度并清除加速度
    if ((dir_sign > 0 && planner->v_curr > planner->v_target) ||
        (dir_sign < 0 && planner->v_curr < planner->v_target)) {
        planner->v_curr = planner->v_target;
        planner->a_curr = 0.0f;
    }

    return planner->v_curr;
}


