/*
 * make_map.c
 *
 *  Created on: 2023/06/11
 *      Author: sato1
 */


#include "queue_class.h"
#include "make_map_class.h"
#include "typedef.h"
#include "index.h"


ring_queue<1024,t_MapNode> maze_q;

t_MapNode node_set(int16_t st_x,int16_t st_y,int16_t cost,int16_t cost_h){
	t_MapNode n;
	n.st_x = st_x;		n.st_y = st_y;
	n.cost = cost;		n.cost_h = cost_h;
	return n;
}

void make_map::init_maze(wall_class *wall_property){
	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			wall_property->wall[i][j].north = UNKNOWN;
			wall_property->wall[i][j].east  = UNKNOWN;
			wall_property->wall[i][j].south = UNKNOWN;
			wall_property->wall[i][j].west  = UNKNOWN;
		}
	}

	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
			wall_property->wall[i][0].south = WALL;				//南側の壁を追加する
			wall_property->wall[i][MAZE_SIZE_Y - 1].north = WALL;	//北側の壁を追加する
	}

	for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
		wall_property->wall[0][j].west = WALL;					//西側の壁を追加する
		wall_property->wall[MAZE_SIZE_X - 1][j].east = WALL;	//東側の壁を追加する
	}

	wall_property->wall[0][0].east = wall_property->wall[1][0].west = WALL;				//スタートの東側の壁を追加

}

void make_map::init_map(int *x, int *y,int goal_size){
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

void make_map::expand(t_MapNode n,int mask,wall_class *wall_property){

	if(n.st_y < MAZE_SIZE_Y-1)					//範囲チェック
	{
		if( (wall_property->wall[n.st_x][n.st_y].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
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
		if( (wall_property->wall[n.st_x][n.st_y].east & mask) == NOWALL)		//壁がなければ
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
		if( (wall_property->wall[n.st_x][n.st_y].south & mask) == NOWALL)	//壁がなければ
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
		if( (wall_property->wall[n.st_x][n.st_y].west & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x-1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x-1][n.st_y] = n.cost + 1;	//値を代入
				maze_q.push(node_set(n.st_x-1,n.st_y,map[n.st_x-1][n.st_y],0));
			}

		}
	}

}

void make_map::make_map_queue(int *x, int *y,t_position expand_end,int size,int mask,wall_class *wall_property)
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
			expand(n,mask,wall_property);
			if(expand_end.x == n.st_x && expand_end.y == n.st_y)
				break;

		}
}


