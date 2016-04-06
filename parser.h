#ifndef PARSER_H
#define PARSER_H
#define COMPILE_FOR_PC
#include "data.h"
#include <queue>

enum Move {
	//1 cell edge
  forward = 0,
  //2 cell edges for straight - 1 cell edge on a diagonal??
  left_90 = 1,
  right_90 = 2,
  //through 3 cell edges
  left_180 = 3,
  right_180 = 4,
  //through 1 cell edge
  setup_left_diag = 5,
  setup_right_diag = 6,
  //through 1 cell edge
  diag = 7,
  //take you through 1 cell edge
  exit_right_diag = 8,
  exit_left_diag = 9,
  //pivots take you through 0 cell edges
  pivot_right_90 = 10,
  pivot_left_90 = 11,
  pivot_180 = 12,
  //assuming 135 take you through 3 cell edges
  right_135 = 13,
  left_135 = 14
};



class FakePath{
  private:
  std::queue<Compass8> fake_path;

  public:
  FakePath(Compass8 path[], int length);
  Compass8 nextDirection();
  Compass8 peek();
  void push(Compass8 item);
  bool isEmpty();
  int getLength();

};

FakePath::FakePath(Compass8 path[], int length){
  int end = length;
  for (int i = 0; i < end; i = i + 1)
  {
    fake_path.push(path[i]);
  }
}

Compass8 FakePath::nextDirection(){
  Compass8 dir = fake_path.front();
  fake_path.pop();
  return dir;
}

void FakePath::push(Compass8 item){
	fake_path.push(item);
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
    Compass8 dir, decision_dir;
    bool lastMoveFlag;
    FakePath path;
    Compass8 relativeDir(Compass8 next_dir, Compass8 current_dir);
    void beginDecision();
    void leftDecisions();
    void rightDecisions();
    void diagonalDecisions(bool approachRight);
    void buildRelativePath(FakePath *abspath);
  public:
   std::queue<int> move_list;
   PathParser(FakePath *abspath);
   std::queue<int> getMoveList();
   std::queue<int> cleanPath();
};

int main(int argc, const char * argv[])
{
	//all-japan 2015 test case: blue
  // Compass8 paddy[] = {kNorth, kEast, kSouth, kEast, kEast, kEast, kEast, kNorth, kEast, kEast, kSouth, kEast, kNorth, kEast, kEast, kSouth,kEast,kEast, kNorth, kEast,
  // 	kSouth, kEast, kEast, kNorth, kWest, kNorth, kNorth, kEast, kNorth, kNorth, kWest, kNorth, kNorth, kEast, kNorth, kWest,
  // 	kWest, kSouth, kWest, kNorth, kWest, kNorth, kWest, kNorth, kNorth, kEast, kSouth, kEast, kEast,
  // 	kNorth, kNorth, kWest, kNorth, kWest, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth,
  // 	kWest, kSouth, kWest, kSouth, kSouth, kEast, kSouth, kEast, kEast, kNorth, kEast,kNorth, kEast, kNorth, kEast, kNorth, kEast, kNorth,
  // 	kWest};

  //all-japan 2015 test case: green


	//diagonal test case
  //Compass8 paddy[] = {kNorth, kEast, kNorth, kEast, kNorth};

  //diagonal cornering test case
  //Compass8 paddy[] = {kEast, kEast, kSouth, kEast, kNorth, kEast, kEast};

  //
  Compass8 paddy[] = {kNorth, kEast, kNorth, kEast, kEast, kEast, kSouth, kEast};
  int length = sizeof(paddy)/sizeof(Compass8);
  std::cout<<length<<"\n";
  FakePath fpath(paddy, length);
  PathParser joe(&fpath);
  joe.getMoveList();
}



PathParser::PathParser(FakePath *abspath) : path(NULL, 0)
{
	lastMoveFlag = false;
  Compass8 movement_direction, next_direction, decision_direction;
  //dir = next_direction = abspath->nextDirection();
  //all paths start by assuming that you're starting from the middle of the previous cell
  buildRelativePath(abspath);
  beginDecision();
}

void PathParser::buildRelativePath(FakePath *abspath){
	//assume moving into the first cell of path
	dir = abspath->nextDirection();
	while(!abspath->isEmpty()){
		Compass8 next_direction = abspath->nextDirection();
    Compass8 rel_dir = relativeDir(next_direction, dir);
    dir = next_direction;
		path.push(rel_dir);
	}
	//final forward into last cell
	//path.push(kNorth);

}

std::queue<int> PathParser::cleanPath(){
	std::queue<int> cleanedPath;
	while(!move_list.empty()){
    int m = move_list.front();
    move_list.pop();
    if(m == right_135 && move_list.front() == setup_left_diag){
    	m = right_90;
    	move_list.pop();
    }
    if(m == left_135 && move_list.front() == setup_right_diag){
    	m = left_90;
    	move_list.pop();
    }
		cleanedPath.push(m);
	}
	return cleanedPath;
}

std::queue<int> PathParser::getMoveList()
{
  //return move_list;
  while(!move_list.empty()){
    int m = move_list.front();
    move_list.pop();
    std::string s;
    switch(m){
      case (forward):
        s = "forward";
        break;
      case (left_90):
        s = "left_90";
        break;
      case (right_90):
        s = "right_90";
        break;
      case (left_180):
        s = "left_180";
        break;
      case(right_180):
        s = "right_180";
        break;
      case(setup_left_diag):
        s = "setup_left_diag";
        break;
      case(setup_right_diag):
        s = "setup_right_diag";
        break;
      case (diag):
        s = "diag";
        break;
      case (exit_right_diag):
        s = "exit_right_diag";
        break;
      case (exit_left_diag):
        s = "exit_left_diag";
        break;
      case (pivot_180):
        s = "pivot_180";
        break;
      case (pivot_left_90):
        s = "pivot_left_90";
        break;
      case (pivot_right_90):
        s = "pivot_right_90";
        break;
      case (right_135):
      	s = "right_135";
      	break;
      case (left_135):
      	s = "left_135";
      	break;
      default:
        break;

    }
    std::cout << s << "\n";
  }
  return move_list;
}

void PathParser::beginDecision(){
  //assumes that every movement into this will be a forward
  while (!path.isEmpty())
  {
    //std::cout<<path->getLength()<<"\n";
    decision_dir = path.nextDirection();

    switch(decision_dir)
    {
      //forward move
      case kNorth:
        move_list.push(forward);
        break;
      //right move
      case kEast:
        rightDecisions();
        break;
      //left move
      case kWest:
        leftDecisions();
        break;
      //if known path is behind you??
      case kSouth:
      	move_list.push(forward);
      	move_list.push(pivot_180);
      	move_list.push(forward);
      default:
        break;
    }
  }

}

void PathParser::rightDecisions(){
	if(path.isEmpty()){
		move_list.push(right_90);
		return;
	}
  decision_dir = path.nextDirection();
  switch(decision_dir){
    //forward move
    case kNorth:
      //set motion corner
      move_list.push(right_90);

      dir = path.peek();
      if(dir == kNorth)
      	move_list.push(forward);

      break;
    //right move
    case kEast:
    	if(path.isEmpty()){
				move_list.push(right_180);
				return;
			}
  		decision_dir = path.nextDirection();

      switch(decision_dir){
        case kNorth:
          //180 outta here
          move_list.push(right_180);
          dir = path.peek();
		      if(dir == kNorth)
		      	move_list.push(forward);
          break;
        case kEast:
          //this is a stupid decision to make because it's just driving a circle, but we'll have to deal with it idk how though
          //just pivot turn to the direction you want to go after going back to original cell
        	if(path.isEmpty())
        		return;
          decision_dir = path.nextDirection();

          switch(decision_dir){
            case(kNorth):
              move_list.push(pivot_left_90);
              move_list.push(forward);
              break;
            case(kWest):
              move_list.push(pivot_180);
              move_list.push(forward);
              break;
            case(kEast):
              move_list.push(forward);
              break;
            default:
              break;
          }

          break;
        case kWest:
        	move_list.push(right_135);
          diagonalDecisions(true);
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
      diagonalDecisions(true);
      break;
    default:
      break;
  }
}

void PathParser::leftDecisions(){
	if(path.isEmpty()){
		move_list.push(left_90);
		return;
	}
  decision_dir = path.nextDirection();

  switch(decision_dir){
    //forward move
    case kNorth:
      //set motion corner
      //dir = kNorth;
      move_list.push(left_90);
      dir = path.peek();
      if(dir == kNorth)
      	move_list.push(forward);
      break;
    //right move
    case kEast:
      //diagonal move
      move_list.push(setup_left_diag);
      move_list.push(diag);
      diagonalDecisions(false);
      break;
    //left move
    case kWest:
      if(path.isEmpty()){
				move_list.push(left_180);
				return;
			}
  		decision_dir = path.nextDirection();

      switch(decision_dir){
        case kNorth:
          //180 outta here
          move_list.push(left_180);
          dir = path.peek();
		      if(dir == kNorth)
		      	move_list.push(forward);
          break;
        case kEast:
        	move_list.push(left_135);
          diagonalDecisions(false);
          break;
        case kWest:
          //just pivot turn to the direction you want to go after going back to original cell
          if(path.isEmpty())
        		return;
          decision_dir = path.nextDirection();

          switch(decision_dir){
            case(kNorth):
              move_list.push(pivot_right_90);
              move_list.push(forward);
              break;
            case(kWest):
              move_list.push(forward);
              break;
            case(kEast):
              move_list.push(pivot_180);
              move_list.push(forward);
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void PathParser::diagonalDecisions(bool approachRight){

	if(path.isEmpty())
  	lastMoveFlag = true;

  if(!lastMoveFlag){
    decision_dir = path.nextDirection();
  }
  else decision_dir = kNorth; //if last move, straighten out

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

        	if(path.isEmpty()){
						move_list.push(exit_right_diag);
						return;
					}
          move_list.push(diag);
          diagonalDecisions(false);
          return;
        }
        else{
        	if(path.isEmpty()){
        		move_list.push(right_135);
        		return;
        	}
        	decision_dir = path.nextDirection();
        	//135 to straighten out or 90 to go to continue diagonal
        	if(decision_dir != kWest)
          	move_list.push(right_135);
          else{
          	move_list.push(right_90);
          	move_list.push(diag);
          	diagonalDecisions(true);
          }
          return;
        }
        break;
      case kWest:
        if(approachRight){
        	if(path.isEmpty()){
        		move_list.push(left_135);
        		return;
        	}
        	decision_dir = path.nextDirection();

        	//135 to straighten out or 90 to go to continue diagonal
        	if(decision_dir != kEast)
          	move_list.push(left_135);
          else{
          	move_list.push(left_90);
          	move_list.push(diag);
          	diagonalDecisions(false);
          }
          return;
        }
        else{

        	if(path.isEmpty()){
						move_list.push(exit_left_diag);
						return;
					}

          move_list.push(diag);
          diagonalDecisions(true);
          return;
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
