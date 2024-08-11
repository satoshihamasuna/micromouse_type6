/*
 * wall_class.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_WALL_CLASS_H_
#define CPP_INC_WALL_CLASS_H_

#include "../../Pheripheral/Include/typedef.h"
#include "../../Pheripheral/Include/macro.h"
#include "../../Task/Inc/sensing_task.h"


class wall_class
{
	IrSensTask *ir_sens;
	public:
		wall_class(IrSensTask *ir_sens_)
		{
			ir_sens = ir_sens_;
		}
		IrSensTask *return_irObj() {return ir_sens;};
		t_wall wall[MAZE_SIZE_X][MAZE_SIZE_Y];
		void init_maze();
		void set_wall(t_position pos);
		t_bool is_unknown(uint16_t x,uint16_t y);
		void goal_set_vwall(int gx,int gy,int goal_size){
			if(goal_size == 3)
			{
				wall[gx+1][gy+1].north = wall[gx+1][gy+1].east = wall[gx+1][gy+1].south = wall[gx+1][gy+1].west = VWALL;
				wall[gx+1][gy+2].south  = wall[gx+2][gy+1].west = wall[gx+1][gy+0].north = wall[gx+0][gy+1].east = VWALL;
			}

		}
		void goal_clear_vwall(int gx,int gy,int goal_size){
			if(goal_size == 3)
			{
				wall[gx+1][gy+1].north = wall[gx+1][gy+1].east = wall[gx+1][gy+1].south = wall[gx+1][gy+1].west = NOWALL;
				wall[gx+1][gy+2].south = wall[gx+2][gy+1].west = wall[gx+1][gy+0].north = wall[gx+0][gy+1].east = NOWALL;
			}
		}
		t_wall_state get_WallState(t_position pos);

};

/*
class wall_class_type7: public wall_class,public Singleton<wall_class_type7>
{
public:
	wall_class_type7(IrSensTask *ir = &IrSensTask_type7::getInstance()):wall_class(ir){}
};
*/

#endif /* CPP_INC_WALL_CLASS_H_ */
