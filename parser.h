#ifndef PARSER_H
#define PARSER_H
#define COMPILE_FOR_PC
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

class FakePath{
	private:
	std::queue<Compass8> fake_path;

	public:
	FakePath(Compass8 path[]);
	Compass8 nextDirection();
	Compass8 peek();
	bool isEmpty();
	int getLength();

};

FakePath::FakePath(Compass8 path[]){
	int end = 16;
	for (int i = 0; i < 16; i = i + 1)
	{
		fake_path.push(path[i]);
	}
}

Compass8 FakePath::nextDirection(){
	Compass8 dir = fake_path.front();
	fake_path.pop();
	return dir;
}

bool FakePath::isEmpty(){
	return fake_path.empty();
}

int FakePath::getLength(){
	return fake_path.size();
}

Compass8 FakePath::peek(){
	return fake_path.front();
}


class PathParser {
  private:
    Compass8 dir;
    Compass8 relativeDir(Compass8 next_dir, Compass8 current_dir);
    void beginDecision(FakePath *path);
    void forwardDecisions(FakePath *path);
    void leftDecisions(FakePath *path);
    void rightDecisions(FakePath *path);
    void diagonalDecisions(bool approachRight, FakePath *path);
  public:
   std::queue<int> move_list;
   PathParser(FakePath *path);
   std::queue<int> getMoveList();
};

int main(int argc, const char * argv[])
{
	std::cout << "HI\n";
	Compass8 paddy[] = {kNorth, kEast, kNorth, kEast, kSouth, kSouth, kSouth, kEast, kNorth, kEast, kNorth, kNorth, kEast, kNorth, kEast, kEast};
	FakePath path(paddy);
	PathParser joe(&path);
	joe.getMoveList();
}



PathParser::PathParser(FakePath *path)
{
  Compass8 movement_direction, next_direction, decision_direction;

  std::cout<<relativeDir(kEast,kSouth)<<"\n";

  // next_direction = path->nextDirection();

  //in the case we got no path
  //if (path->isEmpty())

  //do some pivot logic for the start here

  // movement_direction = next_direction;
  dir = next_direction = path->nextDirection();

  // decision_direction = relativeDir(next_direction, movement_direction);

  //dir = (Compass8)((int)next_direction + (int)decision_direction);

  beginDecision(path);
}

std::queue<int> PathParser::getMoveList()
{
	//return move_list;
	while(!move_list.empty()){
		int m = move_list.front();
		move_list.pop();
		std::cout << m << "\n";
	}
	return move_list;
}

void PathParser::beginDecision(FakePath *path){
	//assumes that every movement into this will be a forward
	while (!path->isEmpty())
	{
		Compass8 next_direction = path->nextDirection();
		Compass8 decision_dir = relativeDir(next_direction, dir);
		dir = next_direction;
		switch(decision_dir)
		{
			//forward move
			case kNorth:
				move_list.push(forward);
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

void PathParser::forwardDecisions(FakePath *path){
	// //dir = (Compass8)(((int)dir + (int)decision_dir)%7);
	// dir = decision_dir;

	// if(decision_dir == kNorth){
	// 	move_list.push(forward);
	// }
	// else
	// 	return;

	// if(path->isEmpty())
	// 	return;
}

void PathParser::rightDecisions(FakePath *path){
	Compass8 next_direction, decision_dir;
	next_direction = path->nextDirection();
	decision_dir = relativeDir(next_direction, dir);
	dir = next_direction;

	switch(decision_dir){
		//forward move
		case kNorth:
			//set motion corner
			move_list.push(right_90);
			break;
		//right move
		case kEast:
			next_direction = path->nextDirection();
			decision_dir = relativeDir(next_direction, dir);
			dir = next_direction;
			switch(decision_dir){
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
					diagonalDecisions(true, path);
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

void PathParser::leftDecisions(FakePath *path){
	Compass8 next_direction, decision_dir;
	next_direction = path->nextDirection();
	decision_dir = relativeDir(next_direction, dir);
	//dir = (Compass8)((int)decision_dir)%7);
	dir = next_direction;

	switch(decision_dir){
		//forward move
		case kNorth:
			//set motion corner
			//dir = kNorth;
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
			next_direction = path->nextDirection();
			decision_dir = relativeDir(next_direction, dir);
			dir = next_direction;
			switch(decision_dir){
				case kNorth:
					//180 outta here
					move_list.push(left_180);
					break;
				case kEast:
					//corner into diagonal special case
					move_list.push(left_90);
					move_list.push(setup_left_diag);
					move_list.push(diag);
					diagonalDecisions(false, path);
					break;
				case kWest:
					break;
				default:
					break;
			}
			break;
		default:
			std::cout<<"SHIT\n";
			break;
	}
}

void PathParser::diagonalDecisions(bool approachRight, FakePath *path){
	Compass8 next_direction = path->peek();
	Compass8 decision_dir = relativeDir(next_direction, dir);
	switch(decision_dir)
		{
			case kNorth:
				if(approachRight)
					move_list.push(exit_left_diag);
				else
					move_list.push(exit_right_diag);

				return;
				break;
			case kEast:
				if(approachRight){
					move_list.push(diag);
					dir = next_direction;
					next_direction = path->nextDirection();
					diagonalDecisions(false, path);
					return;
				}
				else{
					move_list.push(exit_right_diag);
					std::cout<<dir<<"OK\n";
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
					dir = next_direction;
					next_direction = path->nextDirection();
					diagonalDecisions(true, path);
					return;
				}
				break;
			default:
				std::cout<<"SHIT\n";
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