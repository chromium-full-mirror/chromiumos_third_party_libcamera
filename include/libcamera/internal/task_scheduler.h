/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * task_scheduler.h - A task scheduler
 */

#pragma once

#include <list>
#include <map>
#include <vector>

#include <libcamera/base/object.h>
#include <libcamera/base/timer.h>

#include <libcamera/framebuffer.h>

#include "libcamera/internal/framebuffer.h"

namespace libcamera {

class Scheduler;

class Task : public Object
{
public:
	Task(Scheduler* scheduler, const std::string& id = "");
	virtual ~Task() = default;


	virtual void launch();
	virtual void notifyDone();

	virtual void run() = 0;
	std::string &id() { return id_; }

protected:
	Scheduler* scheduler_;
	std::string id_;

private:
	friend Scheduler;

	size_t depend(Task* task);
	size_t removeDependency(Task* task);

	std::list<Task*> dependency_;
};

class DelayedTask : public Task
{
public:
	DelayedTask(std::chrono::milliseconds duration,
		    Scheduler* scheduler,
		    const std::string& id = "");

	virtual void run() override final;

private:
	std::unique_ptr<Timer> timer_;
	std::chrono::milliseconds duration_;
};

class Scheduler : public Object
{
public:
	static void precede(Task* precedent, Task* task);

	Scheduler();

	void schedule();
	void queueTask(Task *task, uint32_t group);
	void taskDone(Task* task);

	std::list<Task*> &groupTasks(uint32_t group);

	Signal<Task*> taskDone_;

protected:
	void succeedPrevTaskByStep(uint32_t group, size_t step, Task* task);

private:
	std::list<std::unique_ptr<Task>> pendingTasks_;
	std::list<std::unique_ptr<Task>> runningTasks_;

	std::map<uint32_t, std::list<Task*>> groupTasks_;
};

template<typename Category, std::enable_if_t<std::is_enum_v<Category>> * = nullptr >
class CategorizedScheduler : public Scheduler
{
public:
	void queueTask(Task *task, Category group) {
		Scheduler::queueTask(task, static_cast<uint32_t>(group));
	}

	std::list<Task*> &groupTasks(Category group) {
		return Scheduler::groupTasks(static_cast<uint32_t>(group));
	}

	void succeedPrevTaskByStep(Category group, size_t step, Task* task) {
		Scheduler::succeedPrevTaskByStep(static_cast<uint32_t>(group), step, task);
	}
};

} /* namespace libcamera */
