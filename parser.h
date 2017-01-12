#ifndef MICROMOUSE_PARSER_H_
#define MICROMOUSE_PARSER_H_

// External libraries
#include <queue>
#include <iostream>

// Dependencies within Micromouse
#include "data.h"

enum Move {
  //half a cell to set up for 90
  half,
  //1 cell edge
  forward,
  //2 cell edges for straight - 1 cell edge on a diagonal??
  left_90,
  right_90,
  //through 3 cell edges
  left_180,
  right_180,
  //through 1 cell edge
  enter_left_45,
  enter_right_45, 
  exit_left_45,
  exit_right_45,
  //through 1 cell edge
  diag,
  //pivots take you through 0 cell edges
  pivot_right_90,
  pivot_left_90,
  pivot_180,
  //assuming 135 take you through 2 cell edges
  //enter is straight to diagonal
  enter_right_135,
  enter_left_135,
  //exit is diagonal to straight
  exit_right_135,
  exit_left_135,
  quarter,
  diag_left_90,
  diag_right_90,
};

struct MoveList{
  Move list[50];
  int index;

  void enqueue(Move item){
    list[index] = item;
    index++;
  }

  Move dequeue(){
    index--;
    return list[index+1];
  }

  Move peek(){
    return list[index];
  }

  int getSize(){
    return index+1;
  }

  bool isEmpty(){
    return index == 0;
  }

  Move* getList(){
    /*Move return_list[index];
    int i = 0;
    for(i =0; i<=index; i++){
      return_list[i] = list[i];
    }
    return return_list;*/
    return list;
  }

};



class FakePath{
  private:
  Queue<Compass8, 50*sizeof(Compass8)> fake_path;

  public:
  FakePath(Compass8 path[], int length);
  Compass8 nextDirection();
  Compass8 peek();
  void push(Compass8 item);
  bool isEmpty();
  int getLength();

};

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
   Queue<Move, 50*sizeof(Move)> move_list;
   PathParser(Path<16, 16> *abspath);
   PathParser(FakePath* fp);

   void getMoveList(Move* array){
    int i = 0;
    while(!move_list.isEmpty()){
      array[i] = move_list.dequeue();
      i++;
    }
   }

   size_t getSize(){
    return move_list.getSize();
   }

   Compass8 getEndDirection();
//   std::queue<int> cleanPath();
    size_t start_x, start_y;
    size_t end_x, end_y;
    Compass8 end_direction;
};

#endif
