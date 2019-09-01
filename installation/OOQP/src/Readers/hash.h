/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/* hash table definitions
 *
 * PCx beta-2.0  10/31/96.
 *
 * Authors: Joe Czyzyk, Sanjay Mehrotra, Steve Wright.
 *
 * (C) 1996 University of Chicago. See COPYRIGHT in main directory.
 */

#ifndef HashFile
#define HashFile

typedef struct node *ListPtr;

typedef struct node {
  int     index;
  char   *entry;
  ListPtr next;
} List;

typedef struct {
  ListPtr *list;
  int      size;
} HashTable;

#ifdef __cplusplus
extern "C" {
#endif
  HashTable  *NewHashTable(int size);
  int Insert (HashTable * table, char * name, int index );
  int GetIndex( HashTable * table, char name[] );
  int DeleteHashTable(HashTable * table);
#ifdef __cplusplus
}
#endif

#endif
