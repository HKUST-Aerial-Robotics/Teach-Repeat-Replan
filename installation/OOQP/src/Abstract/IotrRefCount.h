/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef IOTRREFCOUNT
#define IOTRREFCOUNT

/** @defgroup ReferenceCounting
 * Code to support reference-count garbage collection.
 * @{
 */

/**
 * @file IotrRefCount.h
 *
 * A module supporting reference-count garbage collection.
 **/

#include <cassert>

/**
 * A base class for classes that support reference-count 
 * garbage collection.
 **/
class IotrRefCount {
public:
  /** The number of instances of IotrRefCount. This is useful for
   *  debugging purposes. If not zero when the program exits, you
   *  have a memory leak. */
  static int instances;

  /** The number of references to this object. */
  int refs() { return mRefs; };

  /** Release this reference to obj: it is more convenient to call the
   * template function IotrRelease().  The parameter obj contains the
   * address of a pointer of type exactly (IotrRefCount *). In other
   * words it can't be a pointer to an instance of a
   * subclass. IotrRelease() does not have this restriction.  @see
   * IotrRelease. */
  static inline void release( IotrRefCount ** obj );

  /** Increment the reference count of this object: it is more
   *  convenient to call the template function IotrAddRef().  The
   *  parameter obj contains the address of a pointer of type exactly
   *  (IotrRefCount *). In other words it can't be a pointer to an
   *  instance of a subclass. IotrAddRef() does not have this
   *  restriction. @see IotrAddRef */
  static inline void addRef( IotrRefCount * const * obj );

  /** Default constructor. Creates an instance with one reference */
  IotrRefCount() : mRefs(1) { instances++; };
protected:
  /** Protected virtual destructor. One should not call delete on
   *  reference-counted objects.  
   **/
  virtual ~IotrRefCount() { instances--; };
private:
  /** The number of (hard) references to this object */
  int mRefs;
  /** Operator =. Makes sure that even though all other variables are
   *  copied, the reference count is not! In general, IotrRefCount objects 
   *  should not be copied this way, which is why this operator is private.
   */
  // cppcheck-suppress operatorEqVarError
  IotrRefCount& operator=( const IotrRefCount & )
  {
    // mRefs = mRefs;
    return *this;
  }
  /** Copy constructor. Make sure the reference count for the new object
   *  equals one. In general, IotrRefCount objects should not be copied
   *  this way, which is why this constructor is private. */
  IotrRefCount( const IotrRefCount& )
  {
	instances++;
	mRefs = 1;
  }
};

inline void IotrRefCount::release(IotrRefCount ** obj )
{
  assert( !*obj || (*obj)->mRefs > 0 );
  if( *obj && 0 >= --(*obj)->mRefs ) delete *obj;
  *obj = 0;
}

inline void IotrRefCount::addRef( IotrRefCount * const * obj )
{
  assert( !*obj || (*obj)->mRefs > 0 );
  if( *obj ) (*obj)->mRefs++;
}

/** Release a reference to an object.  If the number of references to
 * obj becomes zero, delete the object.
 *
 * @param obj the address of a pointer to the object to be
 * released. On exit, this pointer (*obj) will be set to nil.
 * */
template <class T>
inline void IotrRelease( T ** obj )
{
  IotrRefCount * objref = *obj;
  IotrRefCount::release( &objref );

  *obj = (T*) objref;
}

/** Increments the reference count of obj.  Objects are only deleted
 * when their reference count becomes zero, so this call must be
 * balanced by a call to IotrRelease to decrement the reference count
 * when this object is no longer needed. 
 *
 * @param obj the address of a pointer to the object whose reference count is 
 * to be incremented. The pointer (*obj) will be unaltered.
 * @see IotrRelease */
template <class T>
inline void IotrAddRef( T * const *  obj )
{
  IotrRefCount * objref = *obj;
  IotrRefCount::addRef( &objref );
}

/** 
 * @}
 */
#endif
