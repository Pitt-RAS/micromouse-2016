#ifndef PARSER_H
#define PARSER_H
#include "data.h"
#include <queue>

enum Move {
	forward = 0,
	left_90 = 1,
	right_90 = 2,
	left_180 = 3,
	right_180 = 4,
	setup_left_diag = 5,
	setup_right_diag = 6,
	diag = 7,
	exit_right_diag = 8,
	exit_left_diag = 9
};

class PathParser {
  private:
    Compass8 dir;
    Compass8 relativeDir(Compass8 next_dir, Compass8 current_dir);
    void beginDecision(Path<16, 16> path);
    void forwardDecisions(Path<16, 16> path);
    void leftDecisions(Path<16, 16> path);
    void rightDecisions(Path<16, 16> path);
    void diagonalDecisions(bool approachRight, Path<16, 16> path);
  public:
   std::queue<int> move_list;
   PathParser(Path<16,16> original_path);
   std::queue<int> getMoveList();
};


PathParser::PathParser(Path<16, 16> path)
{
  Compass8 movement_direction, next_direction, decision_direction;

  next_direction = path.nextDirection();

  //in the case we got no path
  if (path.isEmpty())

  //do some pivot logic for the start here

  movement_direction = next_direction;
  next_direction = path.nextDirection();

  decision_direction = relativeDir(next_direction, movement_direction);

  dir = decision_direction;

  beginDecision(path);
}

std::queue<int> PathParser::getMoveList()
{
	return move_list;
}

void PathParser::beginDecision(Path<16, 16> path){
	//assumes that every movement into this will be a forward
	Compass8 next_direction;
	while (!path.isEmpty())
	{
		switch(dir)
		{
			//forward move
			case kNorth:
				forwardDecisions(path);
				break;
			//right move
			case kEast:
				rightDecisions(path);
				break;
			//left move
			case kWest:
				leftDecisions(path);
				break;
			default:
				break;
		}
	}

}

void PathParser::forwardDecisions(Path<16, 16> path){
	Compass8 next_direction = path.nextDirection();
	Compass8 decision_dir = relativeDir(next_direction, dir);
	dir = decision_dir;

	if(decision_dir == kNorth)
		move_list.push(forward);
	else
		return;

	forwardDecisions(path);
}

void PathParser::rightDecisions(Path<16, 16> path){
	Compass8 next_direction, decision_dir;
	next_direction = path.nextDirection();
	decision_dir = relativeDir(next_direction, dir);
	dir = decision_dir;

	switch(dir){
		//forward move
		case kNorth:
			//set motion corner
			move_list.push(right_90);
			break;
		//right move
		case kEast:
			next_direction = path.nextDirection();
			decision_dir = relativeDir(next_direction, dir);
			dir = decision_dir;
			switch(dir){
				case kNorth:
					//180 outta here
					move_list.push(right_180);
					break;
				case kEast:
					//this is a stupid decision to make because it's just driving a circle, but we'll have to deal with it idk how though
					break;
				case kWest:
					//corner into diagonal special case
					move_list.push(right_90);
					move_list.push(setup_right_diag);
					move_list.push(diag);
					diagonalDecisions(false, path);
					break;
				default:
					break;
			}
			break;
		//left move
		case kWest:
			//diagonal move
			move_list.push(setup_right_diag);
			move_list.push(diag);
			diagonalDecisions(true, path);
			break;
		default:
			break;
	}
}

void PathParser::leftDecisions(Path<16, 16> path){
	Compass8 next_direction, decision_dir;
	next_direction = path.nextDirection();
	decision_dir = relativeDir(next_direction, dir);
	dir = decision_dir;

	switch(dir){
		//forward move
		case kNorth:
			//set motion corner
			move_list.push(left_90);
			break;
		//right move
		case kEast:
			//diagonal move
			move_list.push(setup_left_diag);
			move_list.push(diag);
			diagonalDecisions(false, path);
			break;
		//left move
		case kWest:
			next_direction = path.nextDirection();
			decision_dir = relativeDir(next_direction, dir);
			dir = decision_dir;
			switch(dir){
				case kNorth:
					//180 outta here
					move_list.push(left_180);
					break;
				case kEast:
					//this is a stupid decision to make because it's just driving a circle, but we'll have to deal with it idk how though
					break;
				case kWest:
					//corner into diagonal special case
					move_list.push(left_90);
					move_list.push(setup_left_diag);
					move_list.push(diag);
					diagonalDecisions(false, path);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

void PathParser::diagonalDecisions(bool approachRight, Path<16, 16> path){
	Compass8 next_direction = path.nextDirection();
	Compass8 decision_dir = relativeDir(next_direction, dir);
	dir = decision_dir;
	switch(dir)
		{
			case kNorth:
				//to put us forward again
				if(approachRight)
					move_list.push(exit_left_diag);
				else
					move_list.push(exit_right_diag);
				return;
				break;
			case kEast:
				if(approachRight){
					move_list.push(diag);
					diagonalDecisions(false, path);
				}
				else{
					move_list.push(exit_right_diag);
					return;
				}
				break;
			case kWest:
				if(approachRight){
					move_list.push(exit_left_diag);
					return;
				}
				else{
					move_list.push(diag);
					diagonalDecisions(true, path);
				}
				break;
			default:
				break;
		}
}

Compass8 PathParser::relativeDir(Compass8 next_dir, Compass8 current_dir)
{
  int arc;

  arc = (int) next_dir - (int) current_dir;

  if (arc < 0)
    arc += 8;

  return (Compass8) arc;
}

#endif