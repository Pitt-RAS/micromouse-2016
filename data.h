#ifndef MAZE_H
#define MAZE_H

#ifdef COMPILE_FOR_PC
#include <string>
#include <fstream>
#endif

#ifndef COMPILE_FOR_PC
#include <Arduino.h>
#endif

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
    storage_type *array_[capacity];
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


    // Returns whether or not the queue is empty.
    bool isEmpty();

    // Returns whether or not the queue is full.
    bool isFull();


    // Adds an item to the queue.
    void enqueue(storage_type *item);

    // Returns the next item from the queue and removes the item.
    storage_type *dequeue();

    // Returns the next item from the queue without removing the item.
    storage_type *peek();
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

#ifdef COMPILE_FOR_PC

template <const size_t x_size, const size_t y_size>
void Maze<x_size, y_size>::loadFile(std::string path)
{
  std::ifstream file(path.c_str());
  size_t x, y;
  int n;
  Compass8 direction;

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
void Queue<storage_type, capacity>::enqueue(storage_type *item)
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
storage_type *Queue<storage_type, capacity>::dequeue()
{
  storage_type *item;

  if (isEmpty() || overflowed_)
    return NULL;

  item = array_[front_];
  front_ = indexAfter(front_);
  size_--;

  return item;
}

template <typename storage_type, const size_t capacity>
storage_type *Queue<storage_type, capacity>::peek()
{
  if (isEmpty() || overflowed_)
    return NULL;

  return array_[front_];
}




#endif
