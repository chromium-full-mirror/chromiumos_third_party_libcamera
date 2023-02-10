/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * task_scheduler.cpp - A task scheduler
 */

#include <libcamera/base/log.h>

#include "libcamera/internal/task_scheduler.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(Task)

Task::Task(Scheduler* scheduler, const std::string& id):
	scheduler_(scheduler), id_(id) {}

size_t Task::removeDependency(Task* task)
{
	dependency_.remove(task);
	return dependency_.size();
}

size_t Task::depend(Task* task)
{
	dependency_.emplace_back(task);
	return dependency_.size();
}

void Task::launch()
{
	this->invokeMethod(&Task::run, ConnectionTypeQueued);
}

void Task::notifyDone()
{
	scheduler_->invokeMethod(&Scheduler::taskDone, ConnectionTypeQueued, this);
}

DelayedTask::DelayedTask(std::chrono::milliseconds duration,
			 Scheduler* scheduler, const std::string& id):
	Task(scheduler, id), duration_(duration)
{
	timer_ = std::make_unique<Timer>();
	timer_->timeout.connect(static_cast<Task*>(this), &Task::notifyDone);
}

void DelayedTask::run()
{
	timer_->start(duration_);
}

Scheduler::Scheduler() {}

void Scheduler::precede(Task* precedent, Task* task)
{
	ASSERT(task && precedent);
	task->depend(precedent);
}

void Scheduler::succeedPrevTaskByStep(uint32_t group, size_t step, Task* task)
{
	ASSERT(task);

	auto &tasks = groupTasks_[group];
	if (tasks.size() <= step)
		return;

	auto iter = tasks.rbegin();
	for (size_t i = 0; i < step; i++)
		iter++;

	precede(*iter, task);
}

void Scheduler::schedule()
{
	for (auto it = pendingTasks_.begin(); it != pendingTasks_.end();) {
		if (!(*it)->dependency_.empty()) {
			it++;
			continue;
		}

		auto &task = runningTasks_.emplace_back(std::move(*it));
		it = pendingTasks_.erase(it);

		task->launch();
	}
}

void Scheduler::taskDone(Task* task)
{
	taskDone_.emit(task);

	runningTasks_.remove_if([&task](auto &taskPtr){
			return taskPtr.get() == task; });

	for (auto &[group, tasks] : groupTasks_) {
		tasks.remove(task);
	}

	bool needSchedule = false;
	for (auto &pending : pendingTasks_)
		if (0 == pending->removeDependency(task))
			needSchedule = true;

	if (needSchedule)
		schedule();
}

void Scheduler::queueTask(Task *task, uint32_t group)
{
	/* \todo: Detect cyclic dependency */
	pendingTasks_.emplace_back(task);
	groupTasks_[group].emplace_back(task);
}

std::list<Task*> &Scheduler::groupTasks(uint32_t group)
{
	return groupTasks_[group];
}

} /* namespace libcamera */
