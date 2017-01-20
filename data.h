#ifndef MAZE_H
#define MAZE_H

#ifdef COMPILE_FOR_PC
#include <string>
#include <fstream>
#include <iostream>
//#include <stdio.h>
#endif

//#ifndef COMPILE_FOR_PC
#include <Arduino.h>
//#endif

// A "main 8" compass direction
//   The assigned integers are important. Often times this enum is casted to an
//   integer in order to take advantage of adjacent directions.
enum Compass8 {
  kNorth = 0, kNorthEast = 1, kEast = 2, kSouthEast = 3,
  kSouth = 4, kSouthWest = 5, kWest = 6, kNorthWest = 7
};

// A fixed size maze data structure
//
// A maze is a rectangular grid of boxes. Each box has four walls in the
// kNorth, kSouth, kEast, and kWest Compass8 directions. Each wall either
// exists or does not exist. In addition to this, each box also has a
// visitation state. Each box either has or has not been visited.
//
// Boxes are identified by integer x and y coordinates, with (0, 0) being the
// box in the southwestern corner.
//
// Any two adjacent boxes have one wall in common, which is reflected in the
// state of both boxes. For example, the north wall of a given box is the same
// wall as the south wall of the box to its north. If the north wall of a given
// box exists, then the south wall of the box to its north necessrily also
// exists, because it is the same wall. Similarly, the adding or removing of
// the north wall of a given box is equivalent to the adding or removing of the
// south wall of the box to its north, because both boxes will be affected,
// because both operations occur on the same wall.
//
// The outside border walls of a maze always exist and cannot be removed. When
// a maze is instantiated, all boxes are unvisted, and all walls that are not
// along the outside border do not exist. Any box may be visted or unvisited at
// any time, and any wall that is not an outside border wall may be added or
// removed at any time.
//
// A maze data structure uses x_size times y_size bytes. In other
// words, the number of bytes of memory used is equal to the number of boxes.
//
//   bool return_value;
//   Maze<16, 16> maze;
//
//   maze.addWall(8, 8, kNorth);
//   maze.visit(10, 10);
//
//   return_value = maze.isWall(8, 8, kNorth);  // return_value is true
//   return_value = maze.isWall(8, 8, kSouth);  // return_value is false
//   return_value = maze.isVisited(10, 10);     // return_value is true
//   return_value = maze.isVisited(8, 8);       // return_value is false
//   return_value = maze.isWall(0, 0, kSouth)   // return_value is true
//   return_value = maze.isWall(0, 0, kNorth)   // return_value is false
//
template <const size_t x_size, const size_t y_size>
class Maze
{
  private:
    // A maze box with four walls and a visited flag
    //   The use of bitfields is important for memory efficiency. Each box uses
    //   only one byte.
    struct Box {
      // The four walls are grouped into a single char so that we can use
      // concise code. (Relies on the defined integers in the Compass8 enum.)
      char walls : 4;

      bool visited : 1;
    };

    struct Box boxes_[y_size][x_size];

    // Sets all box flags to false.
    void initializeBoxes();

    // Adds walls around the maze edges.
    void addEdgeWalls();

    // Converts a direction to its bit value in Box.walls
    int dirToBit(Compass8 dir);

    // Returns whether the arguments are valid.
    bool valid(size_t x, size_t y, Compass8 dir);
    bool valid(size_t x, size_t y);

  public:
    Maze();


    // Returns a maze dimension.
    size_t getXSize();
    size_t getYSize();


    // Returns whether or not there is a wall.
    bool isWall(size_t x, size_t y, Compass8 dir);

    // Adds a wall.
    void addWall(size_t x, size_t y, Compass8 dir);

    // Removes a wall.
    void removeWall(size_t x, size_t y, Compass8 dir);

    // Adds a wall if it does not exist, or removes the wall if it does exist.
    void flipWall(size_t x, size_t y, Compass8 dir);


    // Returns whether or not a box has been visited.
    bool isVisited(size_t x, size_t y);

    // Marks a box as visited.
    void visit(size_t x, size_t y);

    // Marks a box as unvisited.
    void unvisit(size_t x, size_t y);

    // Marks all boxes as unvisited.
    void unvisitAll();

    // Puts a string representing this maze into the given buffer
    //
    // This requires a buffer that can hold (4*x + 9*y + 12*x*y + 2)
    // characters, not including the null terminator
    // A 16x16 maze requires 3282 characters
    //
    // Cells look like this:
    //   ___ ___
    //  |   |   |
    //  |   | X |
    //  |___|___|
    //
    //  Visited cells are marked with an X
    void print(char* buf);

#ifdef COMPILE_FOR_PC

    // Loads an existing maze from a file.
    void loadFile(std::string path);

#endif // #ifdef COMPILE_FOR_PC

};

// A generic, fixed capacity, FIFO queue data structure
//
// This data structure stores pointers to data. You must ensure that data does
// not disappear while it is on the queue, or else a dequeued pointer will
// point to data that no longer exists.
//
// This data structure intentionally avoids dynamic memory allocation. The
// queue will _never_ try to allocate or free dynamic memory. Nevertheless, you
// may of course choose to store dynamically allocated data on the queue, but
// remember that you are responsible for managing your own memory.
//
// If the queue is empty, a peek or dequeue operation returns a null pointer.
//
// If the queue is full and an item is added, the queue will become overflowed,
// which is an irreversible state. An overflowed queue will always report that
// it is empty, will always report that it is full, and will always return a
// null pointer for peek and dequeue operations.
//
//   int integer_data_storage;
//   int integer_pointer;
//   int copied_value;
//
//   Queue<int, 10> queue;
//
//   queue.enqueue(&integer_data_storage);
//   integer_pointer = queue.peek();
//   integer_pointer = queue.dequeue();
//   copied_value = *integer_pointer;
//
template <typename storage_type, const size_t capacity>
class Queue
{
  private:
    storage_type array_[capacity];
    size_t front_;
    size_t size_;
    bool overflowed_;

    // Returns the index which is distance items (or the next item) after the
    // given index. The storage array is used as a circular buffer, so this
    // value wraps around after the end of the array.
    size_t indexAfter(size_t index);
    size_t indexAfter(size_t index, size_t distance);

  public:
    Queue();


    // Returns the maximum capacity.
    size_t getCapacity();

    // Returns the number of items in the queue
    size_t getSize();


    // Returns whether or not the queue is empty.
    bool isEmpty();

    // Returns whether or not the queue is full.
    bool isFull();


    // Adds an item to the queue.
    void enqueue(storage_type item);

    // Returns the next item from the queue and removes the item.
    storage_type dequeue();

    // Returns the next item from the queue without removing the item.
    storage_type peek();
};

// Standard interface for a fixed capacity path data structure
//
// A Path is a planned sequence of Compass8 directions leading through a Maze
// between a start location and a finish location. Each direction points to the
// next adjacent cell in a chain of adjacent cells connecting start to finish.
//
// A Path cannot be changed after initialization. The directions of a Path can
// be retrieved only once, individually, in order.
//
// When all directions have been read from a Path, the Path is empty. If a
// direction is read from an empty path, kNorth will always be returned. If
// initial calculation is unsuccessful, a Path will always report that it is
// empty. A calculation may be unsuccessful because of implementation specific
// constraints or because there is no possible traversal between start and
// finish.
//
// The idea is to use a Path reference that points to an instance of a derived
// class. This way, a different derived class can be dropped in without
// changing the surrounding code.
//
// Usually, a number of different connections exist between a given start
// location and a given end location. The derived class is responsible for
// calculating a particular path between start and finish using some algorithm.
// This Path class is responsible for providing the interface.
//
//   DerivedPathClass derived_path<type, size, size>;
//   Path &path = derived_path;
//   Compass8 first_direction;
//   Compass8 second_direction;
//
//   if (!path.isEmpty())
//     first_direction = path.nextDirection();
//     // Do something with first_direction.
//
//   if (!path.isEmpty())
//     second_direction = path.nextDirection();
//     // Do something with second_direction.
//
template <size_t x_size, size_t y_size>
class Path
{
  private:
    bool out_of_range_;
    bool solution_exists_;

  protected:
    // Maze through which the Path is to be calculated
    //
    // This variable shall only be used during calculation. With this
    // restriction, the referenced variable may safely pass out of scope after
    // calculation.
    Maze<x_size, y_size> &maze_;

    // Start and finish maze box coordinates
    size_t start_x_;
    size_t start_y_;
    size_t finish_x_;
    size_t finish_y_;

    // Queue which stores directions in order from start to finish
    Queue<Compass8*, x_size * y_size> directions_;

    // Data to be stored in the directions_ queue
    //
    // [kNorth, kSouth, kEast, kWest]
    Compass8 directions_data_[4];

    // Sets the state to indicate that a solution exists. This is irreversible.
    // If this function is not called, isEmpty() will always return true, and
    // nextDirection() will always return kNorth.
    void setSolutionExists();

  public:
    size_t getStartX();
    size_t getStartY();
    size_t getEndX();
    size_t getEndY();

    // Some functionality must be implemented in a derived class constructor.

    // The derived constructor is responsible for populating the directions_
    // queue with directions. The queue must be populated in order from start
    // to finish.

    // This base Path constructor does not populate the directions_ queue.
    Path(Maze<x_size, y_size> &maze, size_t start_x, size_t start_y,
                                              size_t finish_x, size_t finish_y);

    // Returns whether or not the Path is empty.
    bool isEmpty();

    // Returns the next direction in the Path. The first call returns the first
    // direction, the following call returns the direction to the box after,
    // and so on. The final call returns the direction from the second to last
    // box to the finish.
    Compass8 nextDirection();

};

//stupid getters
template <size_t x_size, size_t y_size>
size_t Path<x_size, y_size>::getStartX() {
  return start_x_;
}
template <size_t x_size, size_t y_size>
size_t Path<x_size, y_size>::getStartY() {
  return start_y_;
}
template <size_t x_size, size_t y_size>
size_t Path<x_size, y_size>::getEndX() {
  return finish_x_;
}
template <size_t x_size, size_t y_size>
size_t Path<x_size, y_size>::getEndY() {
  return finish_y_;
}


// Path that uses the flood fill algorithm
template <size_t x_size, size_t y_size>
class FloodFillPath :
  public Path<x_size, y_size>
{
  public:
    FloodFillPath(Maze<x_size, y_size> &maze, size_t start_x, size_t start_y,
                                              size_t finish_x, size_t finish_y);
};

template <size_t x_size, size_t y_size>
class KnownPath : public Path<x_size, y_size>
{
  public:
    KnownPath(Maze<x_size, y_size> &maze, size_t start_x, size_t start_y,
                                              size_t finish_x, size_t finish_y,
                                              Path<x_size, y_size> &path);
};




template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::initializeBoxes()
{
  size_t x, y;

  for (x = 0; x < x_size; x++)
  for (y = 0; y < y_size; y++) {
    boxes_[y][x].walls = 0;
    boxes_[y][x].visited = false;
  }
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::addEdgeWalls()
{
  size_t i;

  for (i = 0; i < x_size; i++) {
    addWall(i, 0, kSouth);
    addWall(i, y_size - 1, kNorth);
  }

  for (i = 0; i < y_size; i++) {
    addWall(0, i, kWest);
    addWall(x_size - 1, i, kEast);
  }
}

template <const size_t x_size, const size_t y_size>
int Maze<x_size, y_size>::dirToBit(Compass8 dir)
{
  int bits[4];

  bits[0] = 1;
  bits[1] = 2;
  bits[2] = 4;
  bits[3] = 8;

  return bits[(int) dir / 2];
}

template <const size_t x_size, const size_t y_size>
bool Maze<x_size, y_size>::valid(size_t x, size_t y, Compass8 dir)
{
  if (x < 0 || x >= getXSize())
    return false;

  if (y < 0 || y >= getYSize())
    return false;

  if ((int) dir % 2 != 0)
    return false;

  return true;
}

template <const size_t x_size, const size_t y_size>
bool Maze<x_size, y_size>::valid(size_t x, size_t y)
{
  return valid(x, y, kNorth);
}

template <const size_t x_size, const size_t y_size>
Maze<x_size, y_size>::Maze()
{
  initializeBoxes();
  addEdgeWalls();
}

template <const size_t x_size, const size_t y_size>
size_t Maze<x_size, y_size>::getXSize()
{
  return x_size;
}

template <const size_t x_size, const size_t y_size>
size_t Maze<x_size, y_size>::getYSize()
{
  return y_size;
}

template <const size_t x_size, const size_t y_size>
bool Maze<x_size, y_size>::isWall(size_t x, size_t y, Compass8 dir)
{
  if (!valid(x, y, dir))
    return true;

  return boxes_[y][x].walls & dirToBit(dir);
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::addWall(size_t x, size_t y, Compass8 dir)
{
  if (!valid(x, y, dir))
    return;

  boxes_[y][x].walls |= dirToBit(dir);

  switch (dir) {
    case kWest:
      dir = kEast;
      x--;
      break;

    case kEast:
      dir = kWest;
      x++;
      break;

    case kSouth:
      dir = kNorth;
      y--;
      break;

    case kNorth:
      dir = kSouth;
      y++;
      break;

    default:
      break;
  }

  if (!valid(x, y, dir))
    return;

  boxes_[y][x].walls |= dirToBit(dir);
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::removeWall(size_t x, size_t y, Compass8 dir)
{
  if (!valid(x, y, dir))
    return;

  if ((x == 0 && dir == kWest)
      || (x == x_size - 1 && dir == kEast)
      || (y == 0 && dir == kSouth)
      || (y == y_size - 1 && dir == kNorth))
    return;

  boxes_[y][x].walls &= ~dirToBit(dir);

  switch (dir) {
    case kWest:
      dir = kEast;
      x--;
      break;

    case kEast:
      dir = kWest;
      x++;
      break;

    case kSouth:
      dir = kNorth;
      y--;
      break;

    case kNorth:
      dir = kSouth;
      y++;
      break;

    default:
      break;
  }

  if (!valid(x, y, dir))
    return;

  boxes_[y][x].walls &= ~dirToBit(dir);
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::flipWall(size_t x, size_t y, Compass8 dir)
{
  if (!valid(x, y, dir))
    return;

  if (!isWall(x, y, dir))
    addWall(x, y, dir);
  else
    removeWall(x, y, dir);
}

template <const size_t x_size, const size_t y_size>
bool Maze<x_size, y_size>::isVisited(size_t x, size_t y)
{
  if (!valid(x, y))
    return false;

  return boxes_[y][x].visited;
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::visit(size_t x, size_t y)
{
  if (!valid(x, y))
    return;

  boxes_[y][x].visited = true;
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::unvisit(size_t x, size_t y)
{
  if (!valid(x, y))
    return;

  boxes_[y][x].visited = false;
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::unvisitAll()
{
  size_t x, y;

  for (x = 0; x < x_size; x++)
  for (y = 0; y < y_size; y++) {
    unvisit(x, y);
  }
}

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::print(char* buf)
{

  // print top row of maze
  for (int x = 0; x < x_size; x++) {
    *(buf++) = ' ';
    for (int i = 0; i < 3; i++) {
        *(buf++) = '_';
    }
  }
  *(buf++) = '\r';
  *(buf++) = '\n';

  // print rest of maze
  for (size_t y = y_size - 1; y != ((size_t)-1); y--) {
    // print first line of text for this row
    for (int x = 0; x < x_size; x++) {
      if (isWall(x, y, kWest)) {
        *(buf++) = '|';
      } else {
        *(buf++) = ' ';
      }

      for (int i = 0; i < 3; i++) {
        *(buf++) = ' ';
      }
    }
    *(buf++) = '|';
    *(buf++) = '\r';
    *(buf++) = '\n';

    // print second line of text for this row
    for (int x = 0; x < x_size; x++) {
      if (isWall(x, y, kWest)) {
        *(buf++) = '|';
      } else {
        *(buf++) = ' ';
      }

      *(buf++) = ' ';
      *(buf++) = (isVisited(x, y) ? 'X' : ' ');
      *(buf++) = ' ';
    }
    *(buf++) = '|';
    *(buf++) = '\r';
    *(buf++) = '\n';

    // print third line of text for this row
    for (int x = 0; x < x_size; x++) {
      if (isWall(x, y, kWest)) {
        *(buf++) = '|';
      } else {
        *(buf++) = ' ';
      }

      for (int i = 0; i < 3; i++) {
        if (isWall(x, y, kSouth)) {
          *(buf++) = '_';
        } else {
          *(buf++) = ' ';
        }
      }
    }
    *(buf++) = '|';
    *(buf++) = '\r';
    *(buf++) = '\n';
  }

  *(buf++) = '\0';
}

#ifdef COMPILE_FOR_PC

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::loadFile(std::string path)
{
  std::ifstream file(path.c_str());
  size_t x, y;
  int n;
  Compass8 direction;

  if (!file.good()) {
    std::cerr << "Warning: Could not load file `real.maze'" << std::endl
              << "         Continuing without loading a maze." << std::endl;
    return;
  }

  file >> x;
  file >> y;

  if (x != x_size || y != y_size)
    return;

  while (file.good()) {
    file >> x;
    file >> y;
    file >> n;
    direction = (Compass8) (2 * n);

    addWall(x, y, direction);
  }

  file.close();
}

#endif // #ifdef COMPILE_FOR_PC




template <typename storage_type, const size_t capacity>
size_t Queue<storage_type, capacity>::indexAfter(size_t index)
{
  return indexAfter(index, 1);
}

template <typename storage_type, const size_t capacity>
size_t Queue<storage_type, capacity>::indexAfter(size_t index, size_t distance)
{
  return (index + distance) % capacity;
}

template <typename storage_type, const size_t capacity>
Queue<storage_type, capacity>::Queue() : front_(0), size_(0), overflowed_(false)
{}

template <typename storage_type, const size_t capacity>
size_t Queue<storage_type, capacity>::getCapacity()
{
  return capacity;
}

template <typename storage_type, const size_t capacity>
size_t Queue<storage_type, capacity>::getSize(){
  return size_;
}

template <typename storage_type, const size_t capacity>
bool Queue<storage_type, capacity>::isEmpty()
{
  return size_ == 0;
}

template <typename storage_type, const size_t capacity>
bool Queue<storage_type, capacity>::isFull()
{
  return size_ == capacity || overflowed_;
}

template <typename storage_type, const size_t capacity>
void Queue<storage_type, capacity>::enqueue(storage_type item)
{
  if (isFull()) {
    overflowed_ = true;
    size_ = 0;
  }

  if (overflowed_)
    return;

  array_[indexAfter(front_, size_)] = item;
  size_++;
}

template <typename storage_type, const size_t capacity>
storage_type Queue<storage_type, capacity>::dequeue()
{
  storage_type item;

  if (isEmpty() || overflowed_)
    return array_[0];

  item = array_[front_];
  front_ = indexAfter(front_);
  size_--;

  return item;
}

template <typename storage_type, const size_t capacity>
storage_type Queue<storage_type, capacity>::peek()
{
  if (isEmpty() || overflowed_)
    return array_[0];

  return array_[front_];
}




template <size_t x_size, size_t y_size>
void Path<x_size, y_size>::setSolutionExists()
{
  if (!out_of_range_)
    solution_exists_ = true;
}

template <size_t x_size, size_t y_size>
Path<x_size, y_size>::Path(Maze<x_size, y_size> &maze,
                        size_t start_x, size_t start_y,
                        size_t finish_x, size_t finish_y) :
  out_of_range_(false), solution_exists_(false),
  maze_(maze), start_x_(start_x), start_y_(start_y),
  finish_x_(finish_x), finish_y_(finish_y)
{
  if (start_x_ < 0 || start_x_ >= x_size) {
    start_x_ = 0;
    out_of_range_ = true;
  }
  if (start_y_ < 0 || start_y_ >= x_size) {
    start_y_ = 0;
    out_of_range_ = true;
  }
  if (finish_x_ < 0 || finish_x_ >= x_size) {
    finish_x_ = 0;
    out_of_range_ = true;
  }
  if (finish_y_ < 0 || finish_y_ >= x_size) {
    finish_y_ = 0;
    out_of_range_ = true;
  }

  directions_data_[0] = kNorth;
  directions_data_[1] = kSouth;
  directions_data_[2] = kEast;
  directions_data_[3] = kWest;
}

template <size_t x_size, size_t y_size>
bool Path<x_size, y_size>::isEmpty()
{
  if (!solution_exists_)
    return true;

  return directions_.isEmpty();
}

template <size_t x_size, size_t y_size>
Compass8 Path<x_size, y_size>::nextDirection()
{
  if (directions_.isEmpty() || !solution_exists_)
    return kNorth;

  return *directions_.dequeue();
}




template <size_t x_size, size_t y_size>
FloodFillPath<x_size, y_size>::FloodFillPath(
    Maze<x_size, y_size> &maze,
    size_t start_x, size_t start_y,
                size_t finish_x, size_t finish_y) :
    Path<x_size, y_size>(maze,
          start_x, start_y, finish_x, finish_y)
{
  Queue<unsigned char*, x_size * y_size> paint_queue;
  unsigned char boxes_[y_size][x_size];
  size_t x, y;
  bool breaking;
  size_t distance;
  size_t layer_size, next_layer_size;
  unsigned char *item;
  Compass8 *direction_ptr;
  Compass8 *last_direction_ptr = NULL;

  if (this->start_x_ == this->finish_x_ && this->start_y_ == this->finish_y_)
    return;

  for (x = 0; x < x_size; x++) {
    for (y = 0; y < y_size; y++) {
      boxes_[y][x] = 0;
    }
  }

  x = this->finish_x_;
  y = this->finish_y_;
  distance = 0;

  // "Paint" the boxes with distances from the finish.

  paint_queue.enqueue(&boxes_[y][x]);
  next_layer_size = 0;
  layer_size = 1;

  while (!paint_queue.isEmpty()) {
    item = paint_queue.dequeue();
    layer_size--;

    breaking = false;

    for (x = 0; x < x_size; x++) {
      for (y = 0; y < y_size; y++) {
        if (item == &boxes_[y][x])
          breaking = true;

        if (breaking)
          break;
      }

      if (breaking)
        break;
    }

    if (boxes_[y][x] != 0) {
      if (layer_size == 0) {
        distance++;
        layer_size = next_layer_size;
        next_layer_size = 0;
      }
      continue;
    }

    if (y + 1 < y_size && !this->maze_.isWall(x, y, kNorth)
          && boxes_[y + 1][x] == 0) {
      paint_queue.enqueue(&boxes_[y + 1][x]);
      next_layer_size++;
    }

    if (y >= 1 && !this->maze_.isWall(x, y, kSouth)
          && boxes_[y - 1][x] == 0) {
      paint_queue.enqueue(&boxes_[y - 1][x]);
      next_layer_size++;
    }

    if (x + 1 < x_size && !this->maze_.isWall(x, y, kEast)
          && boxes_[y][x + 1] == 0) {
      paint_queue.enqueue(&boxes_[y][x + 1]);
      next_layer_size++;
    }

    if (x >= 1 && !this->maze_.isWall(x, y, kWest)
          && boxes_[y][x - 1] == 0) {
      paint_queue.enqueue(&boxes_[y][x - 1]);
      next_layer_size++;
    }

    if (x == this->finish_x_ && y == this->finish_y_) {
      boxes_[y][x] = 0;
    }
    else {
      boxes_[y][x] = distance;
    }

    if (layer_size == 0) {
      distance++;
      layer_size = next_layer_size;
      next_layer_size = 0;
    }
  }

  x = this->start_x_;
  y = this->start_y_;

  // Stop if no solution was found.
  if (boxes_[y][x] == 0)
    return;

  // Populate the waypoints queue.
  // We do this by always choosing the box with the smallest painted distance.

  while (x != this->finish_x_ || y != this->finish_y_) {
    distance = boxes_[y][x];

    if (y + 1 < y_size && !this->maze_.isWall(x, y, kNorth)
          && boxes_[y + 1][x] < distance) {
      distance = boxes_[y + 1][x];
    }

    if (y >= 1 && !this->maze_.isWall(x, y, kSouth)
          && boxes_[y - 1][x] < distance) {
      distance = boxes_[y - 1][x];
    }

    if (x + 1 < x_size && !this->maze_.isWall(x, y, kEast)
          && boxes_[y][x + 1] < distance) {
      distance = boxes_[y][x + 1];
    }

    if (x >= 1 && !this->maze_.isWall(x, y, kWest)
          && boxes_[y][x - 1] < distance) {
      distance = boxes_[y][x - 1];
    }

    direction_ptr = NULL;

    if (distance != 0) {
      // check the forward direction first, and choose that if it's the same
      //   distance as a turn would be
      if (last_direction_ptr != NULL) {
        switch (*last_direction_ptr) {
          case kNorth: {
            if (y + 1 < y_size && distance == boxes_[y + 1][x]
                  && !this->maze_.isWall(x, y, kNorth)) {
              direction_ptr = &this->directions_data_[0];
              y = y + 1;
            }
            break;
          }
          case kSouth: {
            if (y >= 1 && distance == boxes_[y - 1][x]
                  && !this->maze_.isWall(x, y, kSouth)) {
              direction_ptr = &this->directions_data_[1];
              y = y - 1;
            }
            break;
          }
          case kEast: {
            if (x + 1 < x_size && distance == boxes_[y][x + 1]
                  && !this->maze_.isWall(x, y, kEast)) {
              direction_ptr = &this->directions_data_[2];
              x = x + 1;
            }
            break;
          }
          case kWest: {
            if (x >= 1 && distance == boxes_[y][x - 1]
                  && !this->maze_.isWall(x, y, kWest)) {
              direction_ptr = &this->directions_data_[3];
              x = x - 1;
            }
            break;
          }
          default: {
            // this will never happen, because we set last_direction_ptr ourselves
            break;
          }
        }
      }

      if (direction_ptr == NULL) {
        if (y + 1 < y_size && distance == boxes_[y + 1][x]
              && !this->maze_.isWall(x, y, kNorth)) {
          direction_ptr = &this->directions_data_[0];
          y = y + 1;
        }

        if (y >= 1 && distance == boxes_[y - 1][x]
              && !this->maze_.isWall(x, y, kSouth)) {
          direction_ptr = &this->directions_data_[1];
          y = y - 1;
        }

        if (x + 1 < x_size && distance == boxes_[y][x + 1]
              && !this->maze_.isWall(x, y, kEast)) {
          direction_ptr = &this->directions_data_[2];
          x = x + 1;
        }

        if (x >= 1 && distance == boxes_[y][x - 1]
              && !this->maze_.isWall(x, y, kWest)) {
          direction_ptr = &this->directions_data_[3];
          x = x - 1;
        }
      }
    }
    else {
      if (x == this->finish_x_ && y + 1 == this->finish_y_) {
        direction_ptr = &this->directions_data_[0];
        y = y + 1;
      }
      if (x == this->finish_x_ && y - 1 == this->finish_y_) {
        direction_ptr = &this->directions_data_[1];
        y = y - 1;
      }
      if (x + 1 == this->finish_x_ && y == this->finish_y_) {
        direction_ptr = &this->directions_data_[2];
        x = x + 1;
      }
      if (x - 1 == this->finish_x_ && y == this->finish_y_) {
        direction_ptr = &this->directions_data_[3];
        x = x - 1;
      }
    }

    if (direction_ptr == NULL)
      return;

    last_direction_ptr = direction_ptr;

    this->directions_.enqueue(direction_ptr);
  }

  this->setSolutionExists();
}




template <size_t x_size, size_t y_size>
KnownPath<x_size, y_size>::KnownPath(
    Maze<x_size, y_size> &maze,
    size_t start_x, size_t start_y,
    size_t finish_x, size_t finish_y,
    Path<x_size, y_size> &path) :
    Path<x_size, y_size>(maze,
          start_x, start_y, finish_x, finish_y)
{
  int x, y;
  Compass8 direction;

  x = start_x;
  y = start_y;

  while (!path.isEmpty() && maze.isVisited(x, y)) {
    direction = path.nextDirection();

    switch (direction) {
      case kNorth:
        this->directions_.enqueue(&this->directions_data_[0]);
        y++;
        break;
      case kSouth:
        this->directions_.enqueue(&this->directions_data_[1]);
        y--;
        break;
      case kEast:
        this->directions_.enqueue(&this->directions_data_[2]);
        x++;
        break;
      case kWest:
        this->directions_.enqueue(&this->directions_data_[3]);
        x--;
        break;
      default:
        // do nothing
        break;
    }
  }

  this->setSolutionExists();
}




#endif
