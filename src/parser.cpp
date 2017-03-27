#include "parser.h"

/*namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
    while (1);
  }

  void __throw_length_error(char const* e)
  {
    Serial.print("Length Error :");
    Serial.println(e);
    while (1);
  }
}*/

FakePath::FakePath(Compass8 path[], int length){
  int end = length;
  for (int i = 0; i < end; i = i + 1)
  {
    fake_path.enqueue(path[i]);
  }
}

Compass8 FakePath::nextDirection(){
  Compass8 dir = fake_path.peek();
  fake_path.dequeue();
  return dir;
}

void FakePath::push(Compass8 item){
  fake_path.enqueue(item);
}

bool FakePath::isEmpty(){
  return fake_path.isEmpty();
}

int FakePath::getLength(){
  return (int)fake_path.getSize();
}

Compass8 FakePath::peek(){
  return fake_path.peek();
}

PathParser::PathParser(Path<16, 16> *abspath) : path(NULL, 0)
{
  start_x = abspath->getStartX();
  start_y = abspath->getStartY();
  end_x = abspath->getEndX();
  end_y = abspath->getEndY();

  lastMoveFlag = false;
  //all paths start by assuming that you're starting from the middle of the previous cell
  buildRelativePath(abspath);
  beginDecision();
}

PathParser::PathParser(FakePath* fp) : path(NULL, 0){
  start_x = 0;
  start_y = 0;
  end_x = 8;
  end_y = 8;

  lastMoveFlag = false;

  this->path = *fp;
  beginDecision();
}

void PathParser::buildRelativePath(Path<16, 16> *abspath){
  //assume moving into the first cell of path
  dir = abspath->nextDirection();
  while (!abspath->isEmpty()) {
    Compass8 next_direction = abspath->nextDirection();
    Compass8 rel_dir = relativeDir(next_direction, dir);
    dir = next_direction;
    path.push(rel_dir);
    if(abspath->isEmpty()){
      end_direction = next_direction;
    }
  }
  //final forward into last cell
  //path.push(kNorth);

}

//std::queue<int> PathParser::cleanPath(){
//  std::queue<int> cleanedPath;
//  while(!move_list.empty()){
//    int m = move_list.front();
//    move_list.pop();
//    if(m == right_135 && move_list.front() == setup_left_diag){
//      m = right_90;
//      move_list.pop();
//    }
//    if(m == left_135 && move_list.front() == setup_right_diag){
//      m = left_90;
//      move_list.pop();
//    }
//    cleanedPath.push(m);
//  }
//  return cleanedPath;
//}

Compass8 PathParser::getEndDirection(){
  int size = move_list.getSize();
  int i;
  int angle = 0;
  Move m;
  for(i = 0; i<size;i++){
    m = (Move)move_list.peek();
    move_list.dequeue();
    switch(m){
      case (half):
      case(forward):
      case(quarter):
      case(diag):
        angle+=0;
        break;
      case(left_90):
      case(pivot_left_90):
      case(diag_left_90):
        angle-=90;
        break;
      case(right_90):
      case(pivot_right_90):
      case(diag_right_90):
        angle+=90;
        break;
      case(right_180):
      case(left_180):
      case(pivot_180):
        angle+=180;
        break;
      case(enter_left_45):
      case(exit_left_45):
        angle-=45;
        break;
      case(enter_right_45):
      case(exit_right_45):
        angle+=45;
        break;
      case(enter_left_135):
      case(exit_left_135):
        angle-=135;
        break;
      case(enter_right_135):
      case(exit_right_135):
        angle+=135;
        break;
    }
    move_list.enqueue(m);
  }

  angle%=360;

  if(angle<0)
    angle+=360;

  if (angle == 360)
    angle = 0;

  angle /= 45;

  return (Compass8) angle;
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
        move_list.enqueue(forward);
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
        move_list.enqueue(forward);
        move_list.enqueue(pivot_180);
        move_list.enqueue(forward);
        break;
      default:
        break;
    }
  }
  //if ( condition to be determined )
  //  move_list.enqueue(forward);
}

void PathParser::rightDecisions(){
  if(path.isEmpty()){
    move_list.enqueue(half);
    move_list.enqueue(right_90);
    move_list.enqueue(half);
    return;
  }
  decision_dir = path.nextDirection();
  switch(decision_dir){
    //forward move
    case kNorth:
      //set motion corner
      move_list.enqueue(half);
      move_list.enqueue(right_90);
      move_list.enqueue(half);

      if (detectStraightaway())
      {
        move_list.enqueue(forward);
      }
      break;
    //right move
    case kEast:
      if(path.isEmpty()){
        move_list.enqueue(half);
        move_list.enqueue(right_180);
        move_list.enqueue(half);
        return;
      }
      decision_dir = path.nextDirection();

      switch(decision_dir){
        case kNorth:
          //180 outta here
          move_list.enqueue(half);
          move_list.enqueue(right_180);
          move_list.enqueue(half);
          dir = path.peek();
          if(dir == kNorth){
            move_list.enqueue(forward);
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
              move_list.enqueue(pivot_left_90);
              move_list.enqueue(forward);
              break;
            case(kWest):
              move_list.enqueue(pivot_180);
              move_list.enqueue(forward);
              break;
            case(kEast):
              move_list.enqueue(forward);
              break;
            default:
              break;
          }

          break;
        case kWest:
          move_list.enqueue(enter_right_135);
          move_list.enqueue(diag);
          diagonalDecisions(true);
          break;
        default:
          break;
      }
      break;
    //left move
    case kWest:
      //diagonal move
      move_list.enqueue(enter_right_45);
      move_list.enqueue(diag);
      diagonalDecisions(true);
      break;
    default:
      break;
  }
}

void PathParser::leftDecisions(){
  if(path.isEmpty()){
    move_list.enqueue(half);
    move_list.enqueue(left_90);
    move_list.enqueue(half);
    return;
  }
  decision_dir = path.nextDirection();

  switch(decision_dir){
    //forward move
    case kNorth:
      //set motion corner
      //dir = kNorth;
      move_list.enqueue(half);
      move_list.enqueue(left_90);
      move_list.enqueue(half);
      dir = path.peek();

      if (detectStraightaway())
      {
        move_list.enqueue(forward);
      }
      break;
    //right move
    case kEast:
      //diagonal move
      move_list.enqueue(enter_left_45);
      move_list.enqueue(diag);
      diagonalDecisions(false);
      break;
    //left move
    case kWest:
      if(path.isEmpty()){
                move_list.enqueue(half);
        move_list.enqueue(left_180);
                move_list.enqueue(half);
        return;
      }
      decision_dir = path.nextDirection();

      switch(decision_dir){
        case kNorth:
          //180 outta here
          move_list.enqueue(half);
          move_list.enqueue(left_180);
          move_list.enqueue(half);
          dir = path.peek();
          if(dir == kNorth){
            move_list.enqueue(forward);
            decision_dir = path.nextDirection();
           }
          break;
        case kEast:
          move_list.enqueue(enter_left_135);
          move_list.enqueue(diag);
          diagonalDecisions(false);
          break;
        case kWest:
          //just pivot turn to the direction you want to go after going back to original cell
          if(path.isEmpty())
            return;
          decision_dir = path.nextDirection();

          switch(decision_dir){
            case(kNorth):
              move_list.enqueue(pivot_right_90);
              move_list.enqueue(forward);
              break;
            case(kWest):
              move_list.enqueue(forward);
              break;
            case(kEast):
              move_list.enqueue(pivot_180);
              move_list.enqueue(forward);
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
        if(approachRight) {
          move_list.enqueue(exit_left_45);
        } else {
          move_list.enqueue(exit_right_45);
        }

        if (detectStraightaway() && !lastMoveFlag)
        {
          move_list.enqueue(forward);
        }
        break;
      case kEast:
        if(approachRight){
          move_list.enqueue(diag);
          diagonalDecisions(false);
          return;
        }
        else{
          if(path.isEmpty()){
            move_list.enqueue(exit_right_135);
            return;
          }
          decision_dir = path.nextDirection();
          //135 to straighten out or 90 to go to continue diagonal
          if(decision_dir != kWest) {
            move_list.enqueue(exit_right_135);
          }else{
            move_list.enqueue(diag_right_90);
            move_list.enqueue(diag);
            diagonalDecisions(true);
          }
          return;
        }
        break;
      case kWest:
        if(approachRight){
          if(path.isEmpty()){
            move_list.enqueue(exit_left_135);
            return;
          }
          decision_dir = path.nextDirection();

          //135 to straighten out or 90 to go to continue diagonal
          if(decision_dir != kEast) {
            move_list.enqueue(exit_left_135);
          }else{
            move_list.enqueue(diag_left_90);
            move_list.enqueue(diag);
            diagonalDecisions(false);
          }
          return;
        }
        else{
          move_list.enqueue(diag);
          diagonalDecisions(true);
          return;
        }
        break;
      default:
        break;
    }
}

// determines if the next sequence of moves is a straight away
bool PathParser::detectStraightaway()
{
  bool isStraightawayNext = false;

  Compass8 decision_dir = path.peek();
  while (decision_dir == kNorth && !path.isEmpty())
  {
    path.nextDirection();
    move_list.enqueue(forward);

    decision_dir = path.peek();
  }

  if (path.isEmpty())
  {
    isStraightawayNext = true;
  }

  return isStraightawayNext;
}

Compass8 PathParser::relativeDir(Compass8 next_dir, Compass8 current_dir)
{
  int arc;

  arc = (int) next_dir - (int) current_dir;

  if (arc < 0)
    arc += 8;

  return (Compass8) arc;
}

/*int main(int argc, const char * argv[])
{
  //all-japan 2015 test case: blue
 Compass8 paddy[] = {kNorth, kEast, kSouth, kEast, kEast, kEast, kEast, kNorth, kEast, kEast, kSouth, kEast, kNorth, kEast, kEast, kSouth,kEast,kEast, kNorth, kEast,
  kSouth, kEast, kEast, kNorth, kWest, kNorth, kNorth, kEast, kNorth, kNorth, kWest, kNorth, kNorth, kEast, kNorth, kWest,
  kWest, kSouth, kWest, kNorth, kWest, kNorth, kWest, kNorth, kNorth, kEast, kSouth, kEast, kEast,
  kNorth, kNorth, kWest, kNorth, kWest, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth, kWest, kSouth,
  kWest, kSouth, kWest, kSouth, kSouth, kEast, kSouth, kEast, kEast, kNorth, kEast,kNorth, kEast, kNorth, kEast, kNorth, kEast, kNorth,
  kWest};

//Path<16,16> path;

 // all-japan 2015 test case: green


  // diagonal test case
 //Compass8 paddy[] = {kNorth, kEast, kNorth, kEast, kNorth};

 //diagonal cornering test case
 //Compass8 paddy[] = {kEast, kEast, kSouth, kEast, kNorth, kEast, kEast};

 
 //Compass8 paddy[] = {kNorth, kEast, kNorth};
 int length = sizeof(paddy)/sizeof(Compass8);
 //Serial.println(length);
 FakePath fpath(paddy, length);
 PathParser joe(&fpath);

 Move list[256];
 joe.getMoveList(list);
 int i = 0;
 for(i = 0; i < 256; i++){
  std::cout<<list[i]<<std::endl;
 }

 std::cout<<joe.getSize()<<std::endl;
}
*/
