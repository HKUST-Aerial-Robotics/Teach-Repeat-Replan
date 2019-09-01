/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/* hash tables
 *
 * PCx 1.1 11/97
 *
 * Authors: Joe Czyzyk, Sanjay Mehrotra, Michael Wagner, Steve Wright.
 * 
 * (C) 1996 University of Chicago. See COPYRIGHT in main directory.
 */

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
using namespace std;

// modified 5/21/04 based on the new primes in PCx

#define NUMPRIMES      18
static int prime[NUMPRIMES] = {29, 229, 883, 1669, 2791, 4801, 8629,
                               15289, 32749, 65521, 131071, 262139,
                               524287, 1048573, 2097143, 4194301,
                               8388593, 16777213};


typedef struct node *ListPtr;

typedef struct node {
  int             index;
  char           *entry;
  ListPtr         next;
}               List;

typedef struct {
  ListPtr        *list;
  int             size;
}               HashTable;


extern "C"
void OutOfSpace()
{
  // On newer compilers, can't get here anyway, since new throws and 
  // exception.
  cerr << "Out of Memory!!";
  exit( 1 );
}

extern "C"
char  *
StrDup(char *s1, const char *message)
{
  int ls1 = strlen(s1) + 1;
  char * ptr = new char[ls1];
  strncpy( ptr, s1, ls1 );

  return (ptr);
}

// extern "C"
// HashTable      *NewHashTable();

extern "C"
HashTable      *NewHashTable(int size)
{
  int             i;
  HashTable      *table = 0;

  /* allocate a hash table with size equal to a prime greater than size */
  try {
    table = new HashTable;

    if (size > prime[NUMPRIMES - 1]) {
      printf("The size requested for the hash table is too large: %d.\n",
	     size);
      printf("Either add larger primes to the file 'hash.c' or request\n");
      printf("a smaller table.\n");
      OutOfSpace();
    }
    for (i = 0; i < NUMPRIMES; i++)
      if (size < prime[i]) {
	size = prime[i];
	break;
      }
    table->size = size;
    table->list = 0;
    table->list = new ListPtr[size];
    for (i = 0; i < size; i++)
      table->list[i] = NULL;
    
  } catch (...) {
    if( table ) {
      delete [] table->list;
      delete table;
    }
    cerr << "Could not allocate the table\n";
    throw;
  }
  return (table);
}

/*******************************************************************/

extern "C"
int             hash(HashTable *table, char *string)
{

  unsigned        number, scale;
  char           *s;

  /* Based on the size of the hash table, "hash" converts the string to a
   * number for indexing into the table.  */

  /* 0.618.... = (sqrt(5) - 1) / 2   based on Knuth, v.3, p. 510 */

  scale = (unsigned) (0.6180339887 * table->size);

  number = 0;
  for (s = string; *s != '\0'; s++)
    number = scale * number + *s;
  return (number % table->size);
}

/*******************************************************************/

extern "C"
int             GetIndex(HashTable *table, char *name)
{
  /* Given a name, go through the hash table (down the linked list if
   * necessary) and find the index for the name.  */

  List           *ptr;
  int            match, i;

  /* lookup entry */
  i = hash(table, name);

  match = -1;
  for (ptr = table->list[i]; ptr != NULL; ptr = ptr->next)
    if (strcmp(ptr->entry, name) == 0) {
      match = ptr->index;
      break;
    }
  return (match);
}

/* Insert makes an entry in the hash table.  The entry is indexed on name and
 * also stores an index value.  If name has already been entered into the
 * table, the routine returns a value 1.  */

extern "C"
int Insert(HashTable *table, char *name, int index)
{
  List           *ptr;
  int             i;

  /* lookup entry */

  i = hash(table, name);

  for (ptr = table->list[i]; ptr != NULL; ptr = ptr->next)
    if (strcmp(ptr->entry, name) == 0)
      break;

  if (ptr == NULL) {		/* no entry with "name" was found */
    try {
      ptr = new List;
      ptr->entry = StrDup(name, "entry");
      ptr->index = index;
      
      /* put this entry first in the list */
      ptr->next = table->list[i];
      table->list[i] = ptr;
    } catch ( ... ) {
      cerr << "Not enought memory to insert an item into the hash table";
      throw;
    }
    return (0);			/* normal */
  } else
    return (1);			/* name was already found in table */
}

extern "C"
int PrintHashTable(HashTable *table)
{
  int             i;
  ListPtr         ptr;

  for (i = 0; i < table->size; i++) {
    printf("%d:\n", i);
    for (ptr = table->list[i]; ptr != NULL; ptr = ptr->next)
      printf(" %d '%s'\n", ptr->index, ptr->entry);
  }
  return 0;
}

extern "C"
int DeleteHashTable(HashTable *table)
{
  int             i;
  ListPtr         ptr;

  for (i = 0; i < table->size; i++) {
    ptr = table->list[i];
    while (ptr != NULL) {
      ListPtr next = ptr->next;
      delete [] ptr->entry;
      delete ptr;
      ptr = next;
    }
  }
  delete [] table->list;
  delete table;
  return 0;
}

extern "C"
int PrintHashTableStats(HashTable *table)
{
  int             i, count, max = 0;
  ListPtr         ptr;

  for (i = 0; i < table->size; i++) {
    count = 0;
    for (ptr = table->list[i]; ptr != NULL; ptr = ptr->next)
      count++;

    if (count > max)
      max = count;
  }

  printf("Max size = %d\n", max);
  return 0;
}
