/*
 * wall.cpp
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */



#include "wall_class.h"
#include "../../Module/Include/typedef.h"

void wall_class::init_maze(){
	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			wall[i][j].north = UNKNOWN;
			wall[i][j].east  = UNKNOWN;
			wall[i][j].south = UNKNOWN;
			wall[i][j].west  = UNKNOWN;
		}
	}

	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
			wall[i][0].south = WALL;				//南側の壁を追加する
			wall[i][MAZE_SIZE_Y - 1].north = WALL;	//北側の壁を追加する
	}

	for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			wall[0][j].west = WALL;					//西側の壁を追加する
			wall[MAZE_SIZE_X - 1][j].east = WALL;	//東側の壁を追加する
	}

			wall[0][0].east = wall[1][0].west = WALL;				//スタートの東側の壁を追加

}

t_bool wall_class::is_unknown(uint16_t x,uint16_t y)
{
	if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
	{			//どこかの壁情報が不明のままであれば
		return True;	//未探索
	}
	else
	{
		return False;
	}
}

void wall_class::set_wall(t_position pos)
{
	int n_write,s_write,e_write,w_write;
	n_write = 0;
	s_write = 0;
    e_write = 0;
    w_write = 0;
	//自分の方向に応じて書き込むデータを生成
	//CONV_SEN2WALL()はmacro.hを参照
	switch(pos.dir){
		case North:	//北を向いている時

			n_write = ir_sens->conv_Sensin2Wall(sensor_fr)|ir_sens->conv_Sensin2Wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//　前壁の有無を判断
			e_write = ir_sens->conv_Sensin2Wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右の有無を判断
			w_write = ir_sens->conv_Sensin2Wall(sensor_sl);//CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
			s_write = NOWALL;						//後ろは必ず壁がない

			break;

		case East:	//東を向いているとき

			e_write = ir_sens->conv_Sensin2Wall(sensor_fr)|ir_sens->conv_Sensin2Wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			s_write = ir_sens->conv_Sensin2Wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			n_write = ir_sens->conv_Sensin2Wall(sensor_sl);//				//左壁の有無を判断
			w_write = NOWALL;						//後ろは必ず壁がない

			break;

		case South:	//南を向いているとき

			s_write = ir_sens->conv_Sensin2Wall(sensor_fr)|ir_sens->conv_Sensin2Wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			w_write = ir_sens->conv_Sensin2Wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			e_write = ir_sens->conv_Sensin2Wall(sensor_sl);//				//左壁の有無を判断
			n_write = NOWALL;						//後ろは必ず壁がない

			break;

		case West:	//西を向いているとき

			w_write = ir_sens->conv_Sensin2Wall(sensor_fr)|ir_sens->conv_Sensin2Wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			n_write = ir_sens->conv_Sensin2Wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			s_write = ir_sens->conv_Sensin2Wall(sensor_sl);//				//左壁の有無を判断
			e_write = NOWALL;						//後ろは必ず壁がない

			break;

		case NorthEast:
		case SouthEast:
		case SouthWest:
		case NorthWest:
		case Dir_None:
			break;
	}

	if(wall[pos.x][pos.y].north == UNKNOWN || wall[pos.x][pos.y].north == n_write){
		wall[pos.x][pos.y].north = n_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[pos.x][pos.y].north = VWALL;	//実際に壁情報を書き込み
		n_write			 = VWALL;
	}


	if(wall[pos.x][pos.y].south == UNKNOWN || wall[pos.x][pos.y].south == s_write){
		wall[pos.x][pos.y].south = s_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[pos.x][pos.y].south = VWALL;	//実際に壁情報を書き込み
		s_write			 = VWALL;
	}

	if(wall[pos.x][pos.y].east == UNKNOWN || wall[pos.x][pos.y].east == e_write){
		wall[pos.x][pos.y].east = e_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[pos.x][pos.y].east  = VWALL;	//実際に壁情報を書き込み
		e_write			 = VWALL;
	}

	if(wall[pos.x][pos.y].west == UNKNOWN || wall[pos.x][pos.y].west == w_write){
		wall[pos.x][pos.y].west = w_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[pos.x][pos.y].west  = VWALL;	//実際に壁情報を書き込み
		w_write			 = VWALL;
	}


	//wall[x][y].north = n_write;
	//wall[x][y].south = s_write;	//実際に壁情報を書き込み
	//wall[x][y].east  = e_write;	//実際に壁情報を書き込み
	//wall[x][y].west  = w_write;	//実際に壁情報を書き込み


	if(pos.y < MAZE_SIZE_Y-1)	//範囲チェック
	{
		wall[pos.x][pos.y+1].south = n_write;	//反対側から見た壁を書き込み
	}

	if(pos.x < MAZE_SIZE_X-1)	//範囲チェック
	{
		wall[pos.x+1][pos.y].west = e_write;	//反対側から見た壁を書き込み
	}

	if(pos.y > 0)	//範囲チェック
	{
        wall[pos.x][pos.y-1].north = s_write;	//反対側から見た壁を書き込み
	}

	if(pos.x > 0)	//範囲チェック
	{
		wall[pos.x-1][pos.y].east = w_write;	//反対側から見た壁を書き込み
	}
}

t_wall_state wall_class::get_WallState(t_position pos)
{
	switch(pos.dir)
	{
		case North:
			return (t_wall_state)wall[pos.x][pos.y].north;
		case East:
			return (t_wall_state)wall[pos.x][pos.y].east;
		case South:
			return (t_wall_state)wall[pos.x][pos.y].south;
		case West:
			return (t_wall_state)wall[pos.x][pos.y].west;
		default:
			return UNKNOWN;

	}
}
