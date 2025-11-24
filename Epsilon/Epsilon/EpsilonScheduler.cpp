#include "EpsilonScheduler.h"
void EpsilonScheduler::Run()
{
	for (auto& task : taskQueue) task();
	taskQueue.clear();
}
