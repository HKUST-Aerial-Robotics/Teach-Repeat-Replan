/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef MPSREADER
#define MPSREADER

#include "OoqpVectorHandle.h"
#include "DoubleMatrixHandle.h"
#include "SimpleVectorHandle.h"
#include <cstdio>
#include "hash.h"

struct MpsRowInfo;
struct MpsColInfo;

#ifdef TESTING
class MpsReaderTester;
#endif

/** A class for reading a Quadratic Programming problem from a file
 *  in modified MPS format.
 *
 *  The problem to be read is in QpGen format:
 *  
 *  <pre>
 *  minimize    c' x + ( 1/2 ) x' * Q x         ; 
 *  subject to                      A x  = b    ;
 *                         clow <=  C x <= cupp ;
 *                         xlow <=    x <= xupp ;
 *  </pre>
 *  
 *  The general linear equality constraints must have either an upper
 *  or lower bound, but need not have both bounds. The variables may have 
 *  no bounds; an upper bound; a lower bound or both an upper and lower
 *  bound. 
 */
class MpsReader {
#ifdef TESTING
  friend MpsReaderTester;
#endif
private:
  enum { bufsz = 256, word_max = 31 };

 public:
  typedef char Word[word_max + 1];

 private:
  void insertElt( int irow[], int len, int jcol[],
		  double dval[], int& ne,
		  int row, int col, double val, int& ier );
  void stuffMatrix( GenMatrix& A,
		    int irow[], int nnz, int jcol[], double dA[] );
  void stuffMatrix( SymMatrix& A,
		    int irow[], int nnz, int jcol[], double dA[] );

  /** root of the filename for input (and possibly output) files */
  char * infilename;
  /** Input file line number */
  int iline;

  /** cached values for the number of non-zeros in the parts of this QP */
  int nnzA, nnzC, nnzQ; 
  /** cached values for the sizes of the parts of this QP */
  int my, mz;

  /** the file to be read */
  FILE * file;

  MpsRowInfo * rowInfo;
  int * rowRemap;
  int totalRows;
  
  MpsColInfo * colInfo;
  int totalCols;
  int firstColumnLine;
  int columnFilePosition;

  Word problemName;
  Word objectiveName;
  Word RHSName;
  Word boundName;

  /** hash tables containing row names */
  HashTable *rowTable;

  /** has table containing column names */
  HashTable *colTable;

  double objminus;
  
  /** protected constructor. Call the class method MpsReader::newReadingFile 
   *  to obtain a new, initalized MpsReader. 
   *  @see MpsReader::newReadingFile
   */
  MpsReader( FILE * file );

  /** protected method for scanning the file while initializing the
   *  MpsReader. Once the dimensions of the problem have been
   *  determined, it calls the class method MpsReader::newReadingFile
   *  to obtain an MpsReader with the dimensions appropriately
   *  initialized.
   *
   *  @param iErr iErr is non-zero if there was some error scanning the
   *               file.

   *  @see MpsReader::newReadingFile */
  virtual void scanFile( int& iErr );

  virtual int GetLine(char * line );

  virtual int ParseBoundsLine2( char line[], int& code, char name1[],
			       char name2[], double * val );
  virtual int ParseRowsLine2( char line[], char code[], char name1[] );

  virtual void expectHeader2( int kindOfLine, const char expectName[],
			    char line[], int& ierr );
  virtual void remapRows();

  virtual int acceptHeader2( int kindOfLine, const char expectName[],
			    char line[], int& ierr );
  virtual int ParseDataLine2( char line[], char code[], char name1[], 
                             char name2[], double * val1, int& hasSecondValue,
                             char name3[], double * val2);

  virtual int word_copy( char dest[], char string[]);
  
  virtual int parse_double(const char * token, double & value);

  virtual void readProblemName2( char line[], int& iErr, int kindOfLine );

  virtual void readObjectiveSense( char line[], int& iErr, int kindOfLine );
  virtual void readRowsSection( char line[], 
                int& iErr, int& return_getline );
  virtual void scanColsSection( char line[], 
                int& iErr, int& return_getline );
  virtual void scanRangesSection( char line[], 
                  int& iErr, int& return_getline );
  virtual void rowHasRange( int rownum, double val, int& iErr );
  virtual void scanHessSection( char line[], 
                int& iErr, int& return_getline );

  virtual void readColsSection( OoqpVector& c,
				GenMatrix& A, GenMatrix& C,
				char line[], 
				int& iErr, int& return_getline );
  virtual void readColsSection( double  c[],
			        int irowA[], int jcolA[], double dA[],
				int irowC[], int jcolC[], double dC[],
				char line[], 
				int& iErr, int& return_getline );
  virtual void readRHSSection( OoqpVector&  b,
			       SimpleVector& clow, OoqpVector& iclow,
			       SimpleVector& cupp, OoqpVector& icupp,
			       char line[], int& ierr, int& kindOfLine );
  virtual void readRHSSection( double b[],
			       double clow[], char iclow[],
			       double cupp[], char icupp[],
			       char line[], int& ierr, int& kindOfLine );
  virtual void readRangesSection( SimpleVector& clow, SimpleVector& cupp,
				  char line[], int& ierr, int& kindOfLine );
  virtual void readRangesSection( double clow[], double cupp[],
				  char line[], int& ierr, int& kindOfLine );
  virtual void readBoundsSection( OoqpVector& xlow, OoqpVector& ixlow,
				  OoqpVector& xupp, OoqpVector& ixupp,
				  char line[], int& ierr, int& kindOfLine );
  virtual void defaultBounds( double xlow[], char ixlow[],
			      double xupp[], char ixupp[] );
  virtual void defaultBounds( OoqpVector& xlow, OoqpVector& ixlow,
			      OoqpVector& xupp, OoqpVector& ixupp );
  virtual void readBoundsSection( double xlow[], char ixlow[],
				  double xupp[], char ixupp[],
				  char line[], int& ierr, int& kindOfLine );
  virtual void readHessSection( SymMatrix& Q,
				char line[], int& ierr, int& kindOfLine );
  virtual void readHessSection( int irowQ[], int jcolQ[], double dQ[],
				char line[], int& ierr, int& kindOfLine );

  enum { kMinimize = 0, kMaximize };

  int objectiveSense; /* MAX or MIN */

public:
  /**
   * Objective sense is either MAX or MIN 
   */
  int doMinimize() { return objectiveSense == kMinimize; }
   
  /** Creates a new MpsReader that initializes itself from the data
   *  in a file.
   *  @param filename the name of the file to read. If filename == '-'
   *                  standard input will be read.
   *  @param iErr iErr is non-zero if some error prevented the new
   *              MpsReader from initializing itself, for example if the
   *              file could not be read.
   *  @return the new MpsReader, or nil if there was an error.
   */
  static MpsReader * newReadingFile( char filename[], int& iErr );

  /** 
   * Locate an input file, given the user-supplied name and applying
   * the MpsReader search rules */
  static  void findFile( FILE*& file, char *& resolvedName,
                         char filename[] );
  /**
   * responds with the number of non-zeros in the QP data.
   * @param nnzQ the number of non-zeros in Q
   * @param nnzA the number of non-zeros in A
   * @param nnzC the number of non-zeros in C
   */
  void numberOfNonZeros( int& nnzQ, int& nnzA, int& nnzC );
  /**
   * Reads the various components of a QP in the "general" formulation
   * into their respective matrices and vectors, stored as objects
   * from OOQP's linear algebra classes. See the class comments for
   * the meaning of the variables.  @param iErr iErr is non-zero if
   * there was some error reading the data, in partical if this QP
   * has more than simple bounds.
   */
  virtual void readQpBound( OoqpVector& c, SymMatrix& Q,
			    OoqpVector& xlow, OoqpVector& ixlow,
			    OoqpVector& xupp, OoqpVector& ixupp,
			    int &ierr );
  /**
   * Reads the various components of a QP in the "general" formulation
   * into their respective matrices and vectors, stored as objects
   * from OOQP's linear algebra classes. See the class comments for
   * the meaning of the variables.  @param iErr iErr is non-zero if
   * there was some error reading the data.  */
  virtual void readQpGen( OoqpVector& c, SymMatrix& Q,
			  OoqpVector& xlow, OoqpVector& ixlow,
			  OoqpVector& xupp, OoqpVector& ixupp,
			  GenMatrix& A, OoqpVector& b,
			  GenMatrix& C, 
			  OoqpVector& clow, OoqpVector& iclow,
			  OoqpVector& cupp, OoqpVector& icupp,
			  int& ierr );

  /**
   * Reads the various components of a QP in the "general" formulation
   * into data representaions consisting of arrays of doubles and
   * ints. For instance, the matrices Q, A, and C from this
   * formulation each are represented in three arrays, in
   * Harwell-Boeing format. See the class comments for the meaning of
   * the various variables.  @param iErr iErr is non-zero if there was
   * some error reading the data.  */

  virtual void readQpGen( double     c[],  
			  int    irowQ[],  int  jcolQ[],  double dQ[],
			  double  xlow[],  char ixlow[],
			  double  xupp[],  char ixupp[],
			  int    irowA[],  int  jcolA[],  double dA[],
			  double     b[],
			  int    irowC[],  int  jcolC[],  double dC[],
			  double  clow[],  char  iclow[],
			  double  cupp[],  char  icupp[],
			  int&    ierr );
  /**
   * Returns the sizes of the various components of the QP.
   * @param nx the number of variables
   * @param my the number of equality constraints
   * @param mz the number of general inequality constraints (this number does
   *           not include simple bounds on the variables.)
   */
  virtual void getSizes( int& nx, int& my, int& mz );

  /**
   * Closes the data file if necessary, and forgets all references to
   * it.  Call this method immediately before deleting the MpsReader.
   * We do not close the data file in the destructor, because that
   * operation can fail!
   *
   * Call this method even when reading from stdin. This method will not 
   * close stdin.
   *
   * @param iErr iErr is non-zero if there was some error closing the file.  */
  virtual void releaseFile( int& ierr );
  
  /** Destructor */
  virtual ~MpsReader();

  /////////////////
  // Output section
  /////////////////

  double objconst() { return -objminus; }

  /** print the solution contained in the individual arrays of the
      QpGenVars "variables" object as an ascii file,associating the
      numerical values with the names stored in the MpsReader
      structure */
  void printSolution( double x[],      int     nx,
		      double xlow[],   char    ixlow[],
		      double xupp[],   char    ixupp[],
		      double gamma[],  double  phi[],
		      double y[],      int     my,
		      double s[],      int     mz,
		      double clow[],   char    iclow[],
		      double cupp[],   char    icupp[],
		      double lambda[], double  pi[],
		      double z[],      double  objectiveValue,
		      int& iErr );
  char * defaultOutputFilename( int& iErr );

};

enum { mpsok = 0,
       mpsunknownerr    = -1, // Someone didn't set the error code properly.
       mpsfileopenerr   = -2, // Couldn't open the given file
       mpsioerr         = -3, // There was an i/o error.
       mpssyntaxerr     = -4, // There was a mps syntax error.
       mpsmemoryerr     = -5  // We couldn't allocate the necessary memory
};

#endif
