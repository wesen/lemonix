#ifndef __LINUX_RT_LOCK_H
#define __LINUX_RT_LOCK_H

/*
 * Real-Time Preemption Support
 *
 * started by Ingo Molnar:
 *
 *  Copyright (C) 2004, 2005 Red Hat, Inc., Ingo Molnar <mingo@redhat.com>
 *
 * This file contains the main data structure definitions.
 */
#include <linux/rtmutex.h>
#include <asm/atomic.h>
#include <linux/spinlock_types.h>

#ifdef CONFIG_PREEMPT_RT
/*
 * spinlocks - an RT mutex plus lock-break field:
 */
typedef struct {
	struct rt_mutex		lock;
	unsigned int		break_lock;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
} spinlock_t;

#ifdef CONFIG_DEBUG_RT_MUTEXES
# define __SPIN_LOCK_UNLOCKED(name) \
	(spinlock_t) { { .wait_lock = _RAW_SPIN_LOCK_UNLOCKED(name) \
	, .save_state = 1, .file = __FILE__, .line = __LINE__ }, SPIN_DEP_MAP_INIT(name) }
#else
# define __SPIN_LOCK_UNLOCKED(name) \
	(spinlock_t) { { .wait_lock = _RAW_SPIN_LOCK_UNLOCKED(name) }, SPIN_DEP_MAP_INIT(name) }
#endif
# define SPIN_LOCK_UNLOCKED	__SPIN_LOCK_UNLOCKED(spin_old_style)
#else /* !PREEMPT_RT */
  typedef raw_spinlock_t spinlock_t;
# ifdef CONFIG_DEBUG_SPINLOCK
#  define _SPIN_LOCK_UNLOCKED						\
			{	.raw_lock = __RAW_SPIN_LOCK_UNLOCKED,	\
				.magic = SPINLOCK_MAGIC,		\
				.owner = SPINLOCK_OWNER_INIT,		\
				.owner_cpu = -1 }
# else
#  define _SPIN_LOCK_UNLOCKED \
			{	.raw_lock = __RAW_SPIN_LOCK_UNLOCKED }
# endif
# define SPIN_LOCK_UNLOCKED		_SPIN_LOCK_UNLOCKED
# define __SPIN_LOCK_UNLOCKED(name)	_SPIN_LOCK_UNLOCKED
#endif

#define __DEFINE_SPINLOCK(name) \
	spinlock_t name = __SPIN_LOCK_UNLOCKED(name)

#define DEFINE_SPINLOCK(name) \
	spinlock_t name __cacheline_aligned_in_smp = __SPIN_LOCK_UNLOCKED(name)

#ifdef CONFIG_PREEMPT_RT

/*
 * RW-semaphores are a spinlock plus a reader-depth count.
 *
 * Note that the semantics are different from the usual
 * Linux rw-sems, in PREEMPT_RT mode we do not allow
 * multiple readers to hold the lock at once, we only allow
 * a read-lock owner to read-lock recursively. This is
 * better for latency, makes the implementation inherently
 * fair and makes it simpler as well:
 */
struct rw_semaphore {
	struct rt_mutex		lock;
	int			read_depth;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
};

/*
 * rwlocks - an RW semaphore plus lock-break field:
 */
typedef struct {
	struct rt_mutex		lock;
	int			read_depth;
	unsigned int		break_lock;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
} rwlock_t;

# ifdef CONFIG_DEBUG_RT_MUTEXES
#  define __RW_LOCK_UNLOCKED(name) (rwlock_t) \
	{ .lock = { .wait_lock = _RAW_SPIN_LOCK_UNLOCKED(name), \
	 .save_state = 1, .file = __FILE__, .line = __LINE__ } }
# else
#  define __RW_LOCK_UNLOCKED(name) (rwlock_t) \
	{ .lock = { .wait_lock = _RAW_SPIN_LOCK_UNLOCKED(name) } }
# endif
#else /* !PREEMPT_RT */

  typedef raw_rwlock_t rwlock_t;
# ifdef CONFIG_DEBUG_SPINLOCK
# define _RW_LOCK_UNLOCKED						\
	(rwlock_t)	{	.raw_lock = __RAW_RW_LOCK_UNLOCKED,	\
				.magic = RWLOCK_MAGIC,			\
				.owner = SPINLOCK_OWNER_INIT,		\
				.owner_cpu = -1 }
# else
#  define _RW_LOCK_UNLOCKED						\
	(rwlock_t)	{	.raw_lock = __RAW_RW_LOCK_UNLOCKED }
# endif
# define __RW_LOCK_UNLOCKED(name)	_RW_LOCK_UNLOCKED
#endif

#define RW_LOCK_UNLOCKED	__RW_LOCK_UNLOCKED(rw_old_style)

#define DEFINE_RWLOCK(name) \
	rwlock_t name __cacheline_aligned_in_smp = __RW_LOCK_UNLOCKED(name)

#ifdef CONFIG_PREEMPT_RT

/*
 * Semaphores - a spinlock plus the semaphore count:
 */
struct semaphore {
	atomic_t		count;
	struct rt_mutex		lock;
};

#define DECLARE_MUTEX(name) \
struct semaphore name = \
	{ .count = { 1 }, .lock = __RT_MUTEX_INITIALIZER(name.lock) }

/*
 * DECLARE_MUTEX_LOCKED() is deprecated: very hard to initialize properly
 * and it also often signals abuse of semaphores. So we redirect it to
 * compat semaphores:
 */
#define DECLARE_MUTEX_LOCKED COMPAT_DECLARE_MUTEX_LOCKED

extern void fastcall __sema_init(struct semaphore *sem, int val, char *name, char *file, int line);

#define rt_sema_init(sem, val) \
		__sema_init(sem, val, #sem, __FILE__, __LINE__)

extern void fastcall __init_MUTEX(struct semaphore *sem, char *name, char *file, int line);
#define rt_init_MUTEX(sem) \
		__init_MUTEX(sem, #sem, __FILE__, __LINE__)

extern void there_is_no_init_MUTEX_LOCKED_for_RT_semaphores(void);

/*
 * No locked initialization for RT semaphores
 */
#define rt_init_MUTEX_LOCKED(sem) \
		there_is_no_init_MUTEX_LOCKED_for_RT_semaphores()
extern void fastcall rt_down(struct semaphore *sem);
extern int fastcall rt_down_interruptible(struct semaphore *sem);
extern int fastcall rt_down_trylock(struct semaphore *sem);
extern void fastcall rt_up(struct semaphore *sem);

#define rt_sem_is_locked(s)	rt_mutex_is_locked(&(s)->lock)
#define rt_sema_count(s)	atomic_read(&(s)->count)

extern int __bad_func_type(void);

#undef TYPE_EQUAL
#define TYPE_EQUAL(var, type) \
		__builtin_types_compatible_p(typeof(var), type *)

#define PICK_FUNC_1ARG(type1, type2, func1, func2, arg)			\
do {									\
	if (TYPE_EQUAL((arg), type1))					\
		func1((type1 *)(arg));					\
	else if (TYPE_EQUAL((arg), type2))				\
		func2((type2 *)(arg));					\
	else __bad_func_type();						\
} while (0)

#define PICK_FUNC_1ARG_RET(type1, type2, func1, func2, arg)		\
({									\
	unsigned long __ret;						\
									\
	if (TYPE_EQUAL((arg), type1))					\
		__ret = func1((type1 *)(arg));				\
	else if (TYPE_EQUAL((arg), type2))				\
		__ret = func2((type2 *)(arg));				\
	else __ret = __bad_func_type();					\
									\
	__ret;								\
})

#define PICK_FUNC_2ARG(type1, type2, func1, func2, arg0, arg1)		\
do {									\
	if (TYPE_EQUAL((arg0), type1))					\
		func1((type1 *)(arg0), arg1);				\
	else if (TYPE_EQUAL((arg0), type2))				\
		func2((type2 *)(arg0), arg1);				\
	else __bad_func_type();						\
} while (0)

#define sema_init(sem, val) \
	PICK_FUNC_2ARG(struct compat_semaphore, struct semaphore, \
		compat_sema_init, rt_sema_init, sem, val)

#define init_MUTEX(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_init_MUTEX, rt_init_MUTEX, sem)

#define init_MUTEX_LOCKED(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_init_MUTEX_LOCKED, rt_init_MUTEX_LOCKED, sem)

#define down(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_down, rt_down, sem)

#define down_interruptible(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_down_interruptible, rt_down_interruptible, sem)

#define down_trylock(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_down_trylock, rt_down_trylock, sem)

#define up(sem) \
	PICK_FUNC_1ARG(struct compat_semaphore, struct semaphore, \
		compat_up, rt_up, sem)

#define sem_is_locked(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_sem_is_locked, rt_sem_is_locked, sem)

#define sema_count(sem) \
	PICK_FUNC_1ARG_RET(struct compat_semaphore, struct semaphore, \
		compat_sema_count, rt_sema_count, sem)

/*
 * rwsems:
 */

#define __RWSEM_INITIALIZER(name) \
	{ .lock = __RT_MUTEX_INITIALIZER(name.lock) }

#define DECLARE_RWSEM(lockname) \
	struct rw_semaphore lockname = __RWSEM_INITIALIZER(lockname)

extern void fastcall __rt_rwsem_init(struct rw_semaphore *rwsem, char *name,
				     struct lock_class_key *key);

# define rt_init_rwsem(sem)				\
do {							\
	static struct lock_class_key __key;		\
							\
	__rt_rwsem_init((sem), #sem, &__key);		\
} while (0)

extern void fastcall rt_down_write(struct rw_semaphore *rwsem);
extern void fastcall
rt_down_read_nested(struct rw_semaphore *rwsem, int subclass);
extern void fastcall
rt_down_write_nested(struct rw_semaphore *rwsem, int subclass);
extern void fastcall rt_down_read(struct rw_semaphore *rwsem);
#ifdef CONFIG_DEBUG_LOCK_ALLOC
extern void fastcall rt_down_read_non_owner(struct rw_semaphore *rwsem);
#else
# define rt_down_read_non_owner(rwsem)		rt_down_read(rwsem)
#endif
extern int fastcall rt_down_write_trylock(struct rw_semaphore *rwsem);
extern int fastcall rt_down_read_trylock(struct rw_semaphore *rwsem);
extern void fastcall rt_up_read(struct rw_semaphore *rwsem);
#ifdef CONFIG_DEBUG_LOCK_ALLOC
extern void fastcall rt_up_read_non_owner(struct rw_semaphore *rwsem);
#else
# define rt_up_read_non_owner(rwsem)	rt_up_read(rwsem)
#endif
extern void fastcall rt_up_write(struct rw_semaphore *rwsem);
extern void fastcall rt_downgrade_write(struct rw_semaphore *rwsem);

# define rt_rwsem_is_locked(rws)	(rt_mutex_is_locked(&(rws)->lock))

#define init_rwsem(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_init_rwsem, rt_init_rwsem, rwsem)

#define down_read(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_read, rt_down_read, rwsem)

#define down_read_non_owner(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_read_non_owner, rt_down_read_non_owner, rwsem)

#define down_read_trylock(rwsem) \
	PICK_FUNC_1ARG_RET(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_read_trylock, rt_down_read_trylock, rwsem)

#define down_write(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_write, rt_down_write, rwsem)

#define down_read_nested(rwsem, subclass) \
	PICK_FUNC_2ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_read_nested, rt_down_read_nested, rwsem, subclass)


#define down_write_nested(rwsem, subclass) \
	PICK_FUNC_2ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_write_nested, rt_down_write_nested, rwsem, subclass)

#define down_write_trylock(rwsem) \
	PICK_FUNC_1ARG_RET(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_down_write_trylock, rt_down_write_trylock, rwsem)

#define up_read(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_up_read, rt_up_read, rwsem)

#define up_read_non_owner(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_up_read_non_owner, rt_up_read_non_owner, rwsem)

#define up_write(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_up_write, rt_up_write, rwsem)

#define downgrade_write(rwsem) \
	PICK_FUNC_1ARG(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_downgrade_write, rt_downgrade_write, rwsem)

#define rwsem_is_locked(rwsem) \
	PICK_FUNC_1ARG_RET(struct compat_rw_semaphore, struct rw_semaphore, \
		compat_rwsem_is_locked, rt_rwsem_is_locked, rwsem)

#endif /* CONFIG_PREEMPT_RT */

#endif

