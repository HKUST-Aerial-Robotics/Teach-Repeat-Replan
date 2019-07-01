/** @file dji_memory.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Implement memory management for DJI OSDK .
 *
 *  @copyright 2016-17 DJI. All right reserved.
 *
 */

#ifndef DJI_MEMORY_H
#define DJI_MEMORY_H

#include "dji_type.hpp"

namespace DJI
{
namespace OSDK
{

#define PRO_PURE_DATA_MAX_SIZE 1007 // 2^10 - header size

class MMU
{
public:
  MMU();
  void setupMMU(void);
  void freeMemory(MMU_Tab* mmu_tab);
  MMU_Tab* allocMemory(uint16_t size);

public:
  static const int MMU_TABLE_NUM = 32;
  static const int MEMORY_SIZE   = 1024;

private:
  MMU_Tab memoryTable[MMU_TABLE_NUM];
  uint8_t memory[MEMORY_SIZE];
};

} // OSDK
} // DJI
#endif // DJI_MEMORY_H
