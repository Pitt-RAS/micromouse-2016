#ifndef PARSER_H
#define PARSER_H
#define COMPILE_FOR_PC
#include "data.h"
#include <queue>

#include <Arduino.h>

namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
  }

  void __throw_length_error(char const* e)
  {
    Serial.print("Length Error :");
    Serial.println(e);
  }
}

enum Move {
  //half a cell to set up for 90
  half = 0,
	//1 cell edge
  forward = 1,
  //2 cell edges for straight - 1 cell edge on a diagonal??
  left_90 = 2,
  right_90 = 3,
  //through 3 cell edges
  left_180 = 4,
  right_180 = 5,
  //through 1 cell edge
  left_45 = 6,
  right_45 = 7,
  //through 1 cell edge
  diag = 8,
  //pivots take you through 0 cell edges
  pivot_right_90 = 9,
  pivot_left_90 = 10,
  pivot_180 = 11,
  //assuming 135 take you through 2 cell edges
  right_135 = 12,
  left_135 = 13
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
    void buildRelativePath(Path<16, 16> *abspath);
  public:
   std::queue<int> move_list;
   PathParser(Path<16, 16> *abspath);
   std::queue<int> getMoveList();
   std::queue<int> cleanPath();
};

//int main(int argc, const char * argv[])
//{
//	//all-japan 2015 test case: blue
//  // Compass8 paddy[] = {kNorth, kEast, kSouth, kEast, kEast, kEast, kEast, kNorth, kEast, kEast, kSouth, kEast, kNorth, kEast, kEast, kSouth,kEast,kEast, kNorth, kEast,
//  // 	kSouth, kEast, kEast, kNorth, kWest, kNorth, kNorth, kEast, kNorth, kNorth, kWest, kNorth, kNorth, kEast, kNorth, kWest,
//  // 	kWest, kSouth, kWest, kNorth, kWest, kNorth, kWest, kNorth, kNorth, kEast, kSouth, kEast, kEast,
//  // 	kNorth, kNorth, kWest, kNorth, kWest, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth,
//  // 	kWest, kSouth, kWest, kSouth, kSouth, kEast, kSouth, kEast, kEast, kNorth, kEast,kNorth, kEast, kNorth, kEast, kNorth, kEast, kNorth,
//  // 	kWest};
//
//  //all-japan 2015 test case: green
//
//
//	//diagonal test case
//  //Compass8 paddy[] = {kNorth, kEast, kNorth, kEast, kNorth};
//
//  //diagonal cornering test case
//  //Compass8 paddy[] = {kEast, kEast, kSouth, kEast, kNorth, kEast, kEast};
//
//  //
//  Compass8 paddy[] = {kNorth, kEast, kNorth};
//  int length = sizeof(paddy)/sizeof(Compass8);
//  Serial.println(length);
//  FakePath fpath(paddy, length);
//  PathParser joe(&fpath);
//  joe.getMoveList();
//}



PathParser::PathParser(Path<16, 16> *abspath) : path(NULL, 0)
{
	lastMoveFlag = false;
  Compass8 movement_direction, next_direction, decision_direction;
  //dir = next_direction = abspath->nextDirection();
  //all paths start by assuming that you're starting from the middle of the previous cell
  buildRelativePath(abspath);
  beginDecision();
}

void PathParser::buildRelativePath(Path<16, 16> *abspath){
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
    const char* s;
    switch(m){
      case(half):
        s = "half";
        break;
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
      case(left_45):
        s = "left_45";
        break;
      case(right_45):
        s = "right_45";
        break;
      case (diag):
        s = "diag";
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
    Serial.println(s);
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
		move_list.push(half);
		move_list.push(right_90);
		move_list.push(half);
		return;
	}
  decision_dir = path.nextDirection();
  switch(decision_dir){
    //forward move
    case kNorth:
      //set motion corner
      move_list.push(half);
      move_list.push(right_90);
      move_list.push(half);

      dir = path.peek();
      if(dir == kNorth){
      	move_list.push(forward);
      	decision_dir = path.nextDirection();
     	}
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
		      if(dir == kNorth){
		      	move_list.push(forward);
      			decision_dir = path.nextDirection();
     			}
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
        	move_list.push(diag);
          diagonalDecisions(true);
          break;
        default:
          break;
      }
      break;
    //left move
    case kWest:
      //diagonal move
      move_list.push(right_45);
      move_list.push(diag);
      diagonalDecisions(true);
      break;
    default:
      break;
  }
}

void PathParser::leftDecisions(){
	if(path.isEmpty()){
		move_list.push(half);
		move_list.push(left_90);
		move_list.push(half);
		return;
	}
  decision_dir = path.nextDirection();

  switch(decision_dir){
    //forward move
    case kNorth:
      //set motion corner
      //dir = kNorth;
      move_list.push(half);
      move_list.push(left_90);
      move_list.push(half);
      dir = path.peek();
      if(dir == kNorth){
      	move_list.push(forward);
      	decision_dir = path.nextDirection();
     	}
      break;
    //right move
    case kEast:
      //diagonal move
      move_list.push(left_45);
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
		      if(dir == kNorth){
		      	move_list.push(forward);
      			decision_dir = path.nextDirection();
     			}
          break;
        case kEast:
        	move_list.push(left_135);
        	move_list.push(diag);
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
          move_list.push(left_45);
        else
          move_list.push(left_45);

        return;
        break;
      case kEast:
        if(approachRight){
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
