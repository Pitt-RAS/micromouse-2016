#ifndef PARSER_H
#define PARSER_H
//#define COMPILE_FOR_PC
#include "data.h"
#include <queue>

#include <Arduino.h>

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
  enter_left_45 = 6,
  enter_right_45 = 7, 
  exit_left_45 = 8,
  exit_right_45 = 9,
  //through 1 cell edge
  diag = 10,
  //pivots take you through 0 cell edges
  pivot_right_90 = 11,
  pivot_left_90 = 12,
  pivot_180 = 13,
  //assuming 135 take you through 2 cell edges
  //enter is straight to diagonal
  enter_right_135 = 14,
  enter_left_135 = 15,
  //exit is diagonal to straight
  exit_right_135 = 16,
  exit_left_135 = 17,
  quarter = 18,
  diag_left_90 = 19,
  diag_right_90 = 20
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
   Compass8 getEndDirection();
//   std::queue<int> cleanPath();
    size_t start_x, start_y;
    size_t end_x, end_y;
    Compass8 end_direction;
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

#endif
