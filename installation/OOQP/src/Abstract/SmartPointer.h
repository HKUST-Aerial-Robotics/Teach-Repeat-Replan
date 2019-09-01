/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SMARTPOINTER
#define SMARTPOINTER

/**
 * @file SmartPointer.h
 *
 * A module for supporting automatic management of reference-counted objects.
 * 
 * @ingroup ReferenceCounting
 */
#include <cassert>

/**
 * A class whose instances act like pointers that manage their
 * reference count automatically.
 *
 * Use smart pointers when you wish to keep a reference to a instance of
 * a subclass of IotrRefCount. Do not use a smart pointer to refer to an
 * object passed in as a parameter to a routine, unless you intend to keep 
 * the reference after the routine has exited.
 *
 * Instances of this class are created by value. In other words definitions
 * such as @verbatim SmartPointer<T> t; @endverbatim
 * are encouraged, whereas pointers to objects of type SmartPointer<T> should 
 * not be used.
 *
 * Method calls through a smart pointer look like calls to a regular pointer.
 * In other words
 * @code
 * class T {
 *    public:
 *       void myMethod();
 * };
 * // ...
 * {    
 *   SmartPointer<T> t;
 *    // ...
 *     t->myMethod();
 * }
 * @endcode
 * is the preferred way to call T::myMethod through a smart pointer.
 *
 * SmartPointers can be assigned a pointer to an object through
 * the function SpReferTo(). It is never
 * necessary to release a a SmartPointer, this will be done
 * automatically whenever the SmartPointer goes out of scope. It is
 * not possible, or necessary, to call IotrRelease() or IotrAddRef() on a
 * SmartPointer.
 *
 * Assignments between SmartPointers should work with the same semantics as
 * assigment of traditional pointers.
 * @code
 * SmartPointer<T> t, s;
 * //...
 * t = s;
 * @endcode
 * or
 * @code
 * SmartPointer<T> sp(new T);
 * @endcode
 * or
 * @code
 * T * t = new T;
 * SmartPointer<T> sp;
 * sp = SmartPointer<T>(t); // or sp = (SmartPointer<T>) t;
 * @endcode
 * So long as implicit conversion
 * @code
 * sp = t;
 * @endcode
 * is disallowed.
 * SmartPointers should generally not be in the definition of
 * parameters to routines or as return values. Use traditional
 * pointers for these. Automatic conversion allows SmartPointers to
 * be a "drop-in" replacement to traditional pointers when calling
 * routines.
 * @code
 * class T { // ... 
 * };
 * // ...
 * void myRoutine( T * t );
 * 
 * //...
 * { 
 *     SmartPointer<T> t;
 *     // ...
 *     myRoutine t );
 * }
 * @endcode
 * When returning pointers from a function, be sure to use SpAsPointer()
 * to obtain the traditional pointer to the object.
 * @code
 * class T { // ... 
 * };
 * // ...
 * T * myFunction( T * t );
 * {
 *     SmartPointer<T> t;
 *     // ...
 *     return SpAsPointer( t );
 * }
 * @endcode
 * @see IotrRefCount 
 * @see ReferenceCounting
 * @ingroup ReferenceCounting */
template <class T>
class SmartPointer {
public:
  /** Default constructor; creates a SmartPointer referring to nothing. */
  SmartPointer() : obj(0) {};
  //@{
  /** Copy constructor; creates a new smart pointer referring to the same
   *  object. Increments the reference count of the object.
   *
   *  @param sp an existing reference to an object. This reference may be nil.
   */
  SmartPointer( const SmartPointer<T>& sp )
  {
    this->obj = sp.ptr();
  };
  //@}
  //@{
  /** Dereferencing operation. Return a reference to the object. */
  T& operator*() { return *obj; };
  const T& operator*() const { return *obj; };
  //@}
  //@{
  /** Assignment operator; cause this SmartPointer to refer to the same object
   *  that sp refers to.
   *  @param sp an existing reference to an object. This reference may be nil.
   */
  SmartPointer<T>& operator=( const SmartPointer<T>& sp)
  {
    T * newobj = sp.ptr();
    if( this->obj ) IotrRelease( &this->obj );
    
    this->obj = newobj;
    
    return *this;
  };
  //@}
  /** Destructor; release the object (through a call to IotrRelease()) if the
   *  object is not nil.
   */
  ~SmartPointer()
  {
    if( obj ) IotrRelease( &obj );
  }
  //@{
  /**
   * Send a message to, or access a data memeber of, the object
   * to which this is a reference. */
  T * operator->() { 
    return obj;
  }
  const T * operator->() const { 
    return obj;
  }
  //@}
  /** Automatically convert this object to a traditional pointer to an
   *  object that may be passed in as a paramter to function or method
   *  calls. For most other purposes SmartPointer::ptr() is more appropriate
   */
  operator T*()
  {
    return obj;
  }
  /** Converts a traditional
   *  pointer to a smart pointer, destroying the original pointer. */
  static void bind( SmartPointer<T>& sp, T ** obj )
  {
    sp.obj = *obj;
    *obj   = 0;
  }
  /** Call SpReferTo() instead of this method; make a SmartPointer refer
   *  to the same object as a traditional pointer. */
  static void referTo( SmartPointer<T>& sp, T * obj )
  {
    if( obj )    IotrAddRef( &obj );
    if( sp.obj ) IotrRelease( &sp.obj );
    
    sp.obj = obj;
  }
  /** Call SpAsPointer() instead; return a traditional pointer to the
   *  underlying object.  */
  T * ptr() const
  {
    if (obj) IotrAddRef( &obj );

    return obj;
  }

  /** Allow EXPLICIT conversion from a (T*) to a SmartPointer to
   *  T. Implicit conversion is not allowed, because this could have
   *  subtle, unintended consequences. */
#ifdef HAVE_EXPLICIT
  explicit
#endif 
	SmartPointer( T * t ) { obj = t; }
protected:
  /** a traditional pointer to the object being referenced. */
  T * obj;
};

/**
 * Set a SmartPointer to nil.
 * @ingroup ReferenceCounting */
template <class T>
inline void SpNil( SmartPointer<T>& sp )
{
  T * t = 0;
  SmartPointer<T>::bind( sp, &t );
}

/** Make a SmartPointer refer to the same object as a traditional
 * pointer. The reference count is incremented. Use this function to keep
 * a reference to a parameter of a routine.
 * @ingroup ReferenceCounting */
template <class T>
inline void SpReferTo( SmartPointer<T>& sp, T * obj )
{
  SmartPointer<T>::referTo( sp, obj );
}


/** Call SpAsPointer instead; returns a traditional pointer to the
 *  underlying object. This pointer has its reference count
 *  incremented so that it will continue to exist after the
 *  SmartPointer goes out of scope. Use this method to return a
 *  pointer to the object, or to create a new traditional pointer
 *  reference to the object 
 * @ingroup ReferenceCounting */
template <class T>
inline T * SpAsPointer( SmartPointer<T>& sp )
{
  return sp.ptr();
}

#endif
