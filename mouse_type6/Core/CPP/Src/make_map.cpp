/*
 * make_map.c
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */


#include "queue_class.h"
#include "typedef.h"
#include "index.h"
#include "glob_var_maze.h"

t_MapNode node_set(int16_t st_x,int16_t st_y,int16_t cost,int16_t cost_h){
	t_MapNode n;
	n.st_x = st_x;		n.st_y = st_y;
	n.cost = cost;		n.cost_h = cost_h;
	return n;
}

ring_queue<1024,t_MapNode> maze_q;

void init_maze(){
	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;
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

void init_map(int *x, int *y,int goal_size){
	for( int i = 0; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			map[i][j] = MAZE_SIZE;
		}
	}

	for(int i = 0;i < goal_size;i++){
		for(int j = 0;j < goal_size;j++){
			map[x[i]][y[j]] = 0;
		}
	}

}

void expand(t_MapNode n,int mask){

	if(n.st_y < MAZE_SIZE_Y-1)					//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
		{
			if(map[n.st_x][n.st_y+1] == MAZE_SIZE)			//まだ値が入っていなければ
			{
				map[n.st_x][n.st_y+1] = n.cost + 1;	//値を代入
				maze_q.push(node_set(n.st_x,n.st_y+1,map[n.st_x][n.st_y+1],0));
			}
		}
	}

	if(n.st_x < MAZE_SIZE_X-1)					//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].east & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x+1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x+1][n.st_y] = n.cost + 1;	//値を代入
				maze_q.push(node_set(n.st_x+1,n.st_y,map[n.st_x+1][n.st_y],0));
			}
		}
	}

	if(n.st_y > 0)						//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].south & mask) == NOWALL)	//壁がなければ
		{
			if(map[n.st_x][n.st_y-1] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x][n.st_y-1] = n.cost + 1;	//値を代入
				maze_q.push(node_set(n.st_x,n.st_y-1,map[n.st_x][n.st_y-1],0));
			}
		}
	}

	if(n.st_x > 0)						//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].west & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x-1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x-1][n.st_y] = n.cost + 1;	//値を代入
				maze_q.push(node_set(n.st_x-1,n.st_y,map[n.st_x-1][n.st_y],0));
			}

		}
	}

}

void make_map_queue(int *x, int *y,int size,int mask)
{
	//mapの初期化
		init_map(x,y,size);
	//queueの初期化
		maze_q.queue_reset();

		for(int i = 0;i < size;i++){
			for(int j = 0;j < size;j++){
				maze_q.push(node_set(x[i],y[j],0,0));
			}
		}
	    t_MapNode n;
		while(maze_q.queue_length() != 0){
			n = maze_q.pop();
			expand(n,mask);
			if(mypos.x == n.st_x && mypos.y == n.st_y)
				break;

		}
}

void set_wall(int x, int y)	//壁情報を記録
{
//引数の座標x,yに壁情報を書き込む
	int n_write,s_write,e_write,w_write;
	n_write = 0;
	s_write = 0;
    e_write = 0;
    w_write = 0;
	//自分の方向に応じて書き込むデータを生成
	//CONV_SEN2WALL()はmacro.hを参照
	switch(mypos.dir){
		case North:	//北を向いている時

			n_write = Sensor_is_wall(sensor_fr)|Sensor_is_wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//　前壁の有無を判断
			e_write = Sensor_is_wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右の有無を判断
			w_write = Sensor_is_wall(sensor_sl);//CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
			s_write = NOWALL;						//後ろは必ず壁がない

			break;

		case East:	//東を向いているとき

			e_write = Sensor_is_wall(sensor_fr)|Sensor_is_wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			s_write = Sensor_is_wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			n_write = Sensor_is_wall(sensor_sl);//				//左壁の有無を判断
			w_write = NOWALL;						//後ろは必ず壁がない

			break;

		case South:	//南を向いているとき

			s_write = Sensor_is_wall(sensor_fr)|Sensor_is_wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			w_write = Sensor_is_wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			e_write = Sensor_is_wall(sensor_sl);//				//左壁の有無を判断
			n_write = NOWALL;						//後ろは必ず壁がない

			break;

		case West:	//西を向いているとき

			w_write = Sensor_is_wall(sensor_fr)|Sensor_is_wall(sensor_fl);//CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			n_write = Sensor_is_wall(sensor_sr);//CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			s_write = Sensor_is_wall(sensor_sl);//				//左壁の有無を判断
			e_write = NOWALL;						//後ろは必ず壁がない

			break;

		case NorthEast:
		case SouthEast:
		case SouthWest:
		case NorthWest:
		case Dir_None:
			break;
	}

	if(wall[x][y].north == UNKNOWN || wall[x][y].north == n_write){
		wall[x][y].north = n_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].north = VWALL;	//実際に壁情報を書き込み
		n_write			 = VWALL;
	}


	if(wall[x][y].south == UNKNOWN || wall[x][y].south == s_write){
		wall[x][y].south = s_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].south = VWALL;	//実際に壁情報を書き込み
		s_write			 = VWALL;
	}

	if(wall[x][y].east == UNKNOWN || wall[x][y].east == e_write){
		wall[x][y].east = e_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].east  = VWALL;	//実際に壁情報を書き込み
		e_write			 = VWALL;
	}

	if(wall[x][y].west == UNKNOWN || wall[x][y].west == w_write){
		wall[x][y].west = w_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].west  = VWALL;	//実際に壁情報を書き込み
		w_write			 = VWALL;
	}

	/*
	wall[x][y].north = n_write;
	wall[x][y].south = s_write;	//実際に壁情報を書き込み
	wall[x][y].east  = e_write;	//実際に壁情報を書き込み
	wall[x][y].west  = w_write;	//実際に壁情報を書き込み
	*/

	if(y < MAZE_SIZE_Y-1)	//範囲チェック
	{
		wall[x][y+1].south = n_write;	//反対側から見た壁を書き込み
	}

	if(x < MAZE_SIZE_X-1)	//範囲チェック
	{
		wall[x+1][y].west = e_write;	//反対側から見た壁を書き込み
	}

	if(y > 0)	//範囲チェック
	{
        wall[x][y-1].north = s_write;	//反対側から見た壁を書き込み
	}

	if(x > 0)	//範囲チェック
	{
		wall[x-1][y].east = w_write;	//反対側から見た壁を書き込み
	}

}

void goal_set_vwall(int *gx,int *gy,int goal_size){
	if(goal_size == 3)
	{
		wall[gx[1]][gy[1]].north = wall[gx[1]][gy[1]].east = wall[gx[1]][gy[1]].south = wall[gx[1]][gy[1]].west = VWALL;
		wall[gx[1]][gy[2]].south = wall[gx[2]][gy[1]].west = wall[gx[1]][gy[0]].north = wall[gx[0]][gy[1]].east = VWALL;
	}

}

void goal_clear_vwall(int *gx,int *gy,int goal_size){
	if(goal_size == 3)
	{
		wall[gx[1]][gy[1]].north = wall[gx[1]][gy[1]].east = wall[gx[1]][gy[1]].south = wall[gx[1]][gy[1]].west = NOWALL;
		wall[gx[1]][gy[2]].south = wall[gx[2]][gy[1]].west = wall[gx[1]][gy[0]].north = wall[gx[0]][gy[1]].east = NOWALL;
	}
}

t_bool i_am_goal(int x,int y,int *gx,int *gy,int goal_size){
	t_bool flag = False;
	for (int i = 0; i < goal_size;i++){
		for(int j = 0; j < goal_size;j++){
			if(x == gx[i] && y == gy[j]) flag = True;
		}
	}
	return flag;
}
