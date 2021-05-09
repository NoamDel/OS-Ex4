#include "VirtualMemory.h"
#include "PhysicalMemory.h"
#include <algorithm>


/**
 * Checks if all  table is zeros.
 * @param frameIndex
 * @return  true if  empty table.
 */
bool isAllZeros(uint64_t frameIndex)
{
    word_t val;
    for (uint64_t i = 0; i < PAGE_SIZE; ++i)
    {
        PMread(frameIndex * PAGE_SIZE + i, &val);
        if (val != 0) return false;
    }
    return true;
}

/**
 * Traverses tree and updates the arguments, to find the next table, by the priority rules.
 *
 * @param rootIndex index of current
 * @param depth current depth
 * @param frameNotToErase constant. the frame created just before calling getNextTable.
 * @param frameToSwapIn for calculation cyclic distance
 * @param maxIndex max index of frame used  in physical memory.
 * @param maxDistance  current max cyclic distance
 * @param indexMaxDistance index of frame with max distance
 * @param pageMaxDist PAGE address(route through tree) of  page with max distance
 * @param currentPage PAGE address(route through tree) of current page (only is complete when reach leaf)
 * @return table of zeros to replace. or -1 for condition 2 and 3.
 */
word_t traverseTree(uint64_t rootIndex, int depth, uint64_t frameNotToErase, uint64_t frameToSwapIn, uint64_t *maxIndex,
                    uint64_t *maxDistance, word_t *indexMaxDistance, uint64_t *pageMaxDist, uint64_t currentPage,
                    uint64_t *cyclicFrameFatherAddress)
{
    word_t nextTableIndex;
    uint64_t tableAddress = rootIndex * PAGE_SIZE;
    word_t returnValue;
    if (depth == TABLES_DEPTH)
    {
        //  calculate cyclic distance , and max index:
        for (int i = 0; i < PAGE_SIZE; ++i)
        {
            PMread(tableAddress + i, &nextTableIndex);
            if (nextTableIndex == 0) continue;
            // cyclic distance:
            auto distance = (uint64_t) std::min (std::abs((long long int)(frameToSwapIn - currentPage)),
                    ( (NUM_PAGES - std::abs((long long int)( frameToSwapIn - currentPage)))));
            if (distance >= *maxDistance)
            {
                *maxDistance = distance;
                *indexMaxDistance = nextTableIndex;
                *pageMaxDist = (currentPage << OFFSET_WIDTH) + i;
                *cyclicFrameFatherAddress = tableAddress + i;
            }
            // max index:
            if ((uint64_t) nextTableIndex > *maxIndex)
            { *maxIndex = (uint64_t) nextTableIndex; }
        }
        return -1;
    } else
    {
        for (int i = 0; i < PAGE_SIZE; ++i)
        {
            PMread(tableAddress + i, &nextTableIndex);
            if ((uint64_t) nextTableIndex > *maxIndex)
            { *maxIndex = (uint64_t) nextTableIndex; }
            if (nextTableIndex == 0)
            { continue; }
            if (((uint64_t) nextTableIndex != frameNotToErase) && isAllZeros((uint64_t) nextTableIndex))
            {
                PMwrite(tableAddress + i, 0);
                return nextTableIndex;
            }
            // DFS: go into frame:
            returnValue = traverseTree((uint64_t) nextTableIndex, depth + 1, frameNotToErase, frameToSwapIn, maxIndex,
                                       maxDistance,
                                       indexMaxDistance, pageMaxDist, (currentPage << OFFSET_WIDTH) + i,
                                       cyclicFrameFatherAddress);
            if (returnValue == -1)
            { continue; }
            else return returnValue;

        }
    }
    return -1;
}


/**
 * get a frame to write in, according to priority rules.
 * But do not use 'frameNotToErase'
 * evict a frame if neccessary.
 * @param frameNotToErase not an option ..
 * @param frameToSwapIn bot an option ..
 * @return  a frame index that is free to use. (to clear or restore)
 */
uint64_t getNextTable(uint64_t frameNotToErase, uint64_t frameToSwapIn)
{

    uint64_t maxIndex = 0;
    uint64_t maxDistance = 0;
    word_t indexMaxDist = 0;
    uint64_t pageMaxDist = 0;
    uint64_t currentPage = 0;
    uint64_t cyclicFrameFatherAddress = 0; // if use cyclic (condition 3 ) update father row to 0.

    word_t nextTable = traverseTree(0, 1, frameNotToErase, frameToSwapIn, &maxIndex, &maxDistance, &indexMaxDist,
                                    &pageMaxDist, currentPage, &cyclicFrameFatherAddress);

    if (nextTable > 0)
    {
        return (uint64_t) nextTable; //condition 1;
    }
    if (maxIndex + 1 < NUM_FRAMES)
    {

        return (uint64_t) maxIndex + 1; // condition 2
    }


    PMevict((uint64_t) indexMaxDist, pageMaxDist);
    PMwrite(cyclicFrameFatherAddress, 0); //update father row to 0.

    return (uint64_t) indexMaxDist;

}


void clearTable(uint64_t frameIndex)
{
    for (uint64_t i = 0; i < PAGE_SIZE; ++i)
    {
        PMwrite(frameIndex * PAGE_SIZE + i, 0);
    }
}


void VMinitialize()
{
    clearTable(0);
}


/**
 * Recursive function for reading/writing to RAM memory,
 * @param virtualAddress  full address
 * @param currentVA sub-address(updated recursively)
 * @param value  pointer of value to read/write
 * @param rootFrameIndex  current frame index
 * @param currentDepth current depth in tree
 * @param write true to write. false to read
 * @return 1 on success
 */
int
VMreadWriteHelper(uint64_t virtualAddress, uint64_t currentVA, word_t *value, uint64_t rootFrameIndex, int currentDepth,
                  bool write)
{

    word_t nextTableIndex;
    uint64_t offset = virtualAddress & ((1LL << OFFSET_WIDTH) - 1);
    uint64_t tableAddress = rootFrameIndex * PAGE_SIZE;
    long long rowMask = OFFSET_WIDTH + ((TABLES_DEPTH - currentDepth) * OFFSET_WIDTH);
    uint64_t row = (currentVA >> rowMask);
    PMread(tableAddress + row, &nextTableIndex);
    if (nextTableIndex == 0)
    {
        uint64_t freeFrame = getNextTable(rootFrameIndex, virtualAddress >> OFFSET_WIDTH);

        PMwrite(tableAddress + row, (word_t) freeFrame);
        if (currentDepth == TABLES_DEPTH)
        {
            PMrestore(freeFrame, virtualAddress >> OFFSET_WIDTH);
            if (write)
            { PMwrite(freeFrame * PAGE_SIZE + offset, *value); }
            else
            { PMread(freeFrame * PAGE_SIZE + offset, value); }
            return 1;

        } else
        {
            clearTable(freeFrame);
        }
        return VMreadWriteHelper(virtualAddress, currentVA & ((1LL << rowMask) - 1), value, freeFrame, currentDepth + 1,
                                 write);
    } else
    {
        if (currentDepth == TABLES_DEPTH)
        {
            if (write)
            { PMwrite(nextTableIndex * PAGE_SIZE + offset, *value); }
            else
            { PMread(nextTableIndex * PAGE_SIZE + offset, value); }
            return 1;
        }
        return VMreadWriteHelper(virtualAddress, currentVA & ((1LL << rowMask) - 1), value, (uint64_t) nextTableIndex,
                                 currentDepth + 1, write);
    }
}


int VMread(uint64_t virtualAddress, word_t *value)
{
    if (virtualAddress >= VIRTUAL_MEMORY_SIZE ||  OFFSET_WIDTH >= VIRTUAL_ADDRESS_WIDTH || TABLES_DEPTH + 1 > NUM_FRAMES)
    {
        return 0;
    }
    return VMreadWriteHelper(virtualAddress, virtualAddress, value, 0, 1, false);
}


int VMwrite(uint64_t virtualAddress, word_t value)
{
    if (virtualAddress >= VIRTUAL_MEMORY_SIZE ||  OFFSET_WIDTH >= VIRTUAL_ADDRESS_WIDTH || TABLES_DEPTH + 1 > NUM_FRAMES)
    {
        return 0;
    }
    return VMreadWriteHelper(virtualAddress, virtualAddress, &value, 0, 1, true);
}










