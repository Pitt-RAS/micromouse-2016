#ifndef MATRIX_H
#define MATRIX_H

namespace Motion {

enum class MatrixIndex { a = 0, b = 1, c = 2, d = 3 };

//
// 2x2 matrix
//  _    _
// | a  b |
// |_c  d_|
//
// For convenience and conciseness when operating on an arrangement of
// 2x2 items, e.g. the output applied to four motors.
//
// Typical matrix math operations are not provided, e.g. no dot
// product or cross product.
//
template <typename Type>
class Matrix
{
  public:
    // all zeros
    static Matrix<Type> zeros();

    // all ones
    static Matrix<Type> ones();

    Matrix(Type a, Type b, Type c, Type d);

    // element at index
    Type element(MatrixIndex i) const;
    Type &elementReference(MatrixIndex i);

    // sum of all elements
    Type sum();

    // average of all elements
    Type average();

    // perform scalar operation on each element
    Matrix<Type> add(Type n);
    Matrix<Type> subtract(Type n);
    Matrix<Type> multiply(Type n);
    Matrix<Type> divide(Type n);

    Matrix<Type> add(Matrix<Type> m);
    Matrix<Type> subtract(Matrix<Type> m);
    Matrix<Type> multiply(Matrix<Type> m);
    Matrix<Type> divide(Matrix<Type> m);

    // flip the sign of the left side (a and c)
    Matrix<Type> oppose();

    // subtract away the rotational component (left vs right)
    Matrix<Type> straighten();

    // apply function to each element
    void forEach(void (*function)(Type&));

    // apply function to each element from this and another Matrix
    template <typename OtherType>
    void forEachWith(Matrix<OtherType> other,
                                          void (*function)(Type&, OtherType&));

    // apply function to each element
    template <typename ReturnType>
    Matrix<ReturnType> map(ReturnType (*function)(Type&));

    // apply function to each element from this and another Matrix
    template <typename ReturnType, typename OtherType>
    Matrix<ReturnType> mapWith(Matrix<OtherType> other,
                                    ReturnType (*function)(Type&, OtherType&));

    // apply function to each element from this and two other matrices
    template <typename ReturnType, typename OtherType1, typename OtherType2>
    Matrix<ReturnType> mapWith(Matrix<OtherType1> other1,
                      Matrix<OtherType2> other2,
                      ReturnType (*function)(Type&, OtherType1&, OtherType2&));

  private:
    // because an array of references is illegal
    Type a_, b_, c_, d_;
};

}

namespace Motion {

#define CONSTANT(name, n) \
  template <typename Type> \
  Matrix<Type> Matrix<Type>::name() \
  { \
    return Matrix<Type>((n), (n), (n), (n)); \
  } \

CONSTANT(zeros, 0)
CONSTANT(ones, 1)

template <typename Type>
Matrix<Type>::Matrix(Type a, Type b, Type c, Type d) :
  a_(a), b_(b), c_(c), d_(d)
{}

template <typename Type>
Type Matrix<Type>::element(MatrixIndex i) const
{
  switch (i) {
    case MatrixIndex::a: return a_;
    case MatrixIndex::b: return b_;
    case MatrixIndex::c: return c_;
    case MatrixIndex::d: return d_;
    default: return a_;
  };
}

template <typename Type>
Type &Matrix<Type>::elementReference(MatrixIndex i)
{
  switch (i) {
    case MatrixIndex::a: return a_;
    case MatrixIndex::b: return b_;
    case MatrixIndex::c: return c_;
    case MatrixIndex::d: return d_;
    default: return a_;
  };
}

#define SUM (a_ + b_ + c_ + d_)

template <typename Type>
Type Matrix<Type>::sum()
{
  return SUM;
}

template <typename Type>
Type Matrix<Type>::average()
{
  return SUM / 4.0;
}

#define SCALAR_OPERATION(name, operation) \
  template <typename Type> \
  Matrix<Type> Matrix<Type>::name(Type n) \
  { \
    return Matrix<Type>(a_ operation n, b_ operation n, \
                        c_ operation n, d_ operation n); \
  }

SCALAR_OPERATION(add, +);
SCALAR_OPERATION(subtract, -);
SCALAR_OPERATION(multiply, *);
SCALAR_OPERATION(divide, /);

#define MATRIX_OPERATION(name, operation) \
  template <typename Type> \
  Matrix<Type> Matrix<Type>::name(Matrix<Type> m) \
  { \
    return Matrix<Type>(a_ operation m.element(MatrixIndex::a), \
                        b_ operation m.element(MatrixIndex::b), \
                        c_ operation m.element(MatrixIndex::c), \
                        d_ operation m.element(MatrixIndex::d)); \
  }

MATRIX_OPERATION(add, +);
MATRIX_OPERATION(subtract, -);
MATRIX_OPERATION(multiply, *);
MATRIX_OPERATION(divide, /);

template <typename Type>
Matrix<Type> Matrix<Type>::oppose()
{
  return Matrix<Type>(-a_, b_, -c_, d_);
}

template <typename Type>
Matrix<Type> Matrix<Type>::straighten()
{
  Type rotation = (-a_ + b_ + -c_ + d_) / 4.0;

  return Matrix<Type>(
    a_ + rotation,
    b_ - rotation,
    c_ + rotation,
    d_ - rotation
  );
}

template <typename Type>
void Matrix<Type>::forEach(void (*function)(Type&))
{
  function(this->element(MatrixIndex::a));
  function(this->element(MatrixIndex::b));
  function(this->element(MatrixIndex::c));
  function(this->element(MatrixIndex::d));
}

template <typename Type>
template <typename OtherType>
void Matrix<Type>::forEachWith(Matrix<OtherType> other,
                                          void (*function)(Type&, OtherType&))
{
  function(
    this->elementReference(MatrixIndex::a),
    other.elementReference(MatrixIndex::a)
  );
  function(
    this->elementReference(MatrixIndex::b),
    other.elementReference(MatrixIndex::b)
  );
  function(
    this->elementReference(MatrixIndex::c),
    other.elementReference(MatrixIndex::c)
  );
  function(
    this->elementReference(MatrixIndex::d),
    other.elementReference(MatrixIndex::d)
  );
}

template <typename Type>
template <typename ReturnType>
Matrix<ReturnType> Matrix<Type>::map(ReturnType (*function)(Type&))
{
  return Matrix<ReturnType>(
    function(this->elementReference(MatrixIndex::a)),
    function(this->elementReference(MatrixIndex::b)),
    function(this->elementReference(MatrixIndex::c)),
    function(this->elementReference(MatrixIndex::d))
  );
}

template <typename Type>
template <typename ReturnType, typename OtherType>
Matrix<ReturnType> Matrix<Type>::mapWith(Matrix<OtherType> other,
                                    ReturnType (*function)(Type&, OtherType&))
{
  return Matrix<ReturnType>(
    function(
      this->elementReference(MatrixIndex::a),
      other.elementReference(MatrixIndex::a)
    ),
    function(
      this->elementReference(MatrixIndex::b),
      other.elementReference(MatrixIndex::b)
    ),
    function(
      this->elementReference(MatrixIndex::c),
      other.elementReference(MatrixIndex::c)
    ),
    function(
      this->elementReference(MatrixIndex::d),
      other.elementReference(MatrixIndex::d)
    )
  );
}

template <typename Type>
template <typename ReturnType, typename OtherType1, typename OtherType2>
Matrix<ReturnType> Matrix<Type>::mapWith(Matrix<OtherType1> other1,
                      Matrix<OtherType2> other2,
                      ReturnType (*function)(Type&, OtherType1&, OtherType2&))
{
  return Matrix<ReturnType>(
    function(
      this->elementReference(MatrixIndex::a),
      other1.elementReference(MatrixIndex::a),
      other2.elementReference(MatrixIndex::a)
    ),
    function(
      this->elementReference(MatrixIndex::b),
      other1.elementReference(MatrixIndex::b),
      other2.elementReference(MatrixIndex::b)
    ),
    function(
      this->elementReference(MatrixIndex::c),
      other1.elementReference(MatrixIndex::c),
      other2.elementReference(MatrixIndex::c)
    ),
    function(
      this->elementReference(MatrixIndex::d),
      other1.elementReference(MatrixIndex::d),
      other2.elementReference(MatrixIndex::d)
    )
  );
}

}

#endif
