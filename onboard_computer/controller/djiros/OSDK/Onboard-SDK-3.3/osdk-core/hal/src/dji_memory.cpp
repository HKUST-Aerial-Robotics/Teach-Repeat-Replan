/** @file dji_memory.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Implement memory management for OSDK library
 *
 *  @copyright 2016-17 DJI. All right reserved.
 *
 *
 *  @attention
 *  It is not necessary to include dji_memory.hpp in any custom code file.
 *  The functions in this file are not API functions.
 *  Do not modify this file if you are unsure about it.
 *
 */

#include "dji_memory.hpp"
#include <string.h>

using namespace DJI::OSDK;

MMU::MMU()
{
}

void
MMU::setupMMU()
{
  uint32_t i;
  memoryTable[0].tabIndex  = 0;
  memoryTable[0].usageFlag = 1;
  memoryTable[0].pmem      = memory;
  memoryTable[0].memSize   = 0;
  for (i = 1; i < (MMU_TABLE_NUM - 1); i++)
  {
    memoryTable[i].tabIndex  = i;
    memoryTable[i].usageFlag = 0;
  }
  memoryTable[MMU_TABLE_NUM - 1].tabIndex  = MMU_TABLE_NUM - 1;
  memoryTable[MMU_TABLE_NUM - 1].usageFlag = 1;
  memoryTable[MMU_TABLE_NUM - 1].pmem      = memory + MEMORY_SIZE;
  memoryTable[MMU_TABLE_NUM - 1].memSize   = 0;
}

void
MMU::freeMemory(MMU_Tab* mmu_tab)
{
  if (mmu_tab == (MMU_Tab*)0)
    return;
  if (mmu_tab->tabIndex == 0 || mmu_tab->tabIndex == (MMU_TABLE_NUM - 1))
    return;
  mmu_tab->usageFlag = 0;
}

MMU_Tab*
MMU::allocMemory(uint16_t size)
{
  uint32_t mem_used = 0;
  uint8_t  i;
  uint8_t  j                = 0;
  uint8_t  mmu_tab_used_num = 0;
  uint8_t  mmu_tab_used_index[MMU_TABLE_NUM];

  uint32_t temp32;
  uint32_t temp_area[2] = { 0xFFFFFFFF, 0xFFFFFFFF };

  uint32_t record_temp32 = 0;
  uint8_t  magic_flag    = 0;

  if (size > PRO_PURE_DATA_MAX_SIZE || size > MEMORY_SIZE)
    return (MMU_Tab*)0;

  for (i = 0; i < MMU_TABLE_NUM; i++)
    if (memoryTable[i].usageFlag == 1)
    {
      mem_used += memoryTable[i].memSize;
      mmu_tab_used_index[mmu_tab_used_num++] = memoryTable[i].tabIndex;
    }

  if (MEMORY_SIZE < (mem_used + size))
    return (MMU_Tab*)0;

  if (mem_used == 0)
  {
    memoryTable[1].pmem      = memoryTable[0].pmem;
    memoryTable[1].memSize   = size;
    memoryTable[1].usageFlag = 1;
    return &memoryTable[1];
  }

  for (i = 0; i < (mmu_tab_used_num - 1); i++)
    for (j = 0; j < (mmu_tab_used_num - i - 1); j++)
      if (memoryTable[mmu_tab_used_index[j]].pmem >
          memoryTable[mmu_tab_used_index[j + 1]].pmem)
      {
        mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
        mmu_tab_used_index[j] ^= mmu_tab_used_index[j + 1];
        mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
      }

  for (i = 0; i < (mmu_tab_used_num - 1); i++)
  {
    temp32 = static_cast<uint32_t>(memoryTable[mmu_tab_used_index[i + 1]].pmem -
                                   memoryTable[mmu_tab_used_index[i]].pmem);

    if ((temp32 - memoryTable[mmu_tab_used_index[i]].memSize) >= size)
    {
      if (temp_area[1] > (temp32 - memoryTable[mmu_tab_used_index[i]].memSize))
      {
        temp_area[0] = memoryTable[mmu_tab_used_index[i]].tabIndex;
        temp_area[1] = temp32 - memoryTable[mmu_tab_used_index[i]].memSize;
      }
    }

    record_temp32 += temp32 - memoryTable[mmu_tab_used_index[i]].memSize;
    if (record_temp32 >= size && magic_flag == 0)
    {
      j          = i;
      magic_flag = 1;
    }
  }

  if (temp_area[0] == 0xFFFFFFFF && temp_area[1] == 0xFFFFFFFF)
  {
    for (i = 0; i < j; i++)
    {
      if (memoryTable[mmu_tab_used_index[i + 1]].pmem >
          (memoryTable[mmu_tab_used_index[i]].pmem +
           memoryTable[mmu_tab_used_index[i]].memSize))
      {
        memmove(memoryTable[mmu_tab_used_index[i]].pmem +
                  memoryTable[mmu_tab_used_index[i]].memSize,
                memoryTable[mmu_tab_used_index[i + 1]].pmem,
                memoryTable[mmu_tab_used_index[i + 1]].memSize);
        memoryTable[mmu_tab_used_index[i + 1]].pmem =
          memoryTable[mmu_tab_used_index[i]].pmem +
          memoryTable[mmu_tab_used_index[i]].memSize;
      }
    }

    for (i = 1; i < (MMU_TABLE_NUM - 1); i++)
    {
      if (memoryTable[i].usageFlag == 0)
      {
        memoryTable[i].pmem = memoryTable[mmu_tab_used_index[j]].pmem +
                              memoryTable[mmu_tab_used_index[j]].memSize;

        memoryTable[i].memSize   = size;
        memoryTable[i].usageFlag = 1;
        return &memoryTable[i];
      }
    }
    return (MMU_Tab*)0;
  }

  for (i = 1; i < (MMU_TABLE_NUM - 1); i++)
  {
    if (memoryTable[i].usageFlag == 0)
    {
      memoryTable[i].pmem =
        memoryTable[temp_area[0]].pmem + memoryTable[temp_area[0]].memSize;

      memoryTable[i].memSize   = size;
      memoryTable[i].usageFlag = 1;
      return &memoryTable[i];
    }
  }

  return (MMU_Tab*)0;
}
