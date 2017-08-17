/*
 * Userspace implementations of gettimeofday() and friends.
 *
 * Copyright (C) 2017 Cavium, Inc.
 * Copyright (C) 2015 Mentor Graphics Corporation
 * Copyright (C) 2012 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Will Deacon <will.deacon@arm.com>
 * Rewriten from arch64 version into C by: Andrew Pinski <apinski@cavium.com>
 * Reworked and rebased over arm version by: Mark Salyzyn <salyzyn@android.com>
 */

<<<<<<< HEAD
#include <linux/compiler.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <asm/barrier.h>
#include <asm/bug.h>
#include <asm/cp15.h>
#include <asm/page.h>
#include <asm/unistd.h>
=======
#include <asm/barrier.h>
#include <linux/compiler.h>	/* for notrace				*/
#include <linux/math64.h>	/* for __iter_div_u64_rem()		*/
#include <uapi/linux/time.h>	/* for struct timespec			*/
>>>>>>> a790acc4b21a... FROMLIST: [PATCH v5 04/12] arm: vdso: do calculations outside reader loops

#ifndef CONFIG_AEABI
#error This code depends on AEABI system call conventions
#endif

#include "datapage.h"

static notrace u32 vdso_read_begin(const struct vdso_data *vd)
{
	u32 seq;

	do {
		seq = READ_ONCE(vd->tb_seq_count);

		if ((seq & 1) == 0)
			break;

		cpu_relax();
	} while (true);

	smp_rmb(); /* Pairs with second smp_wmb in update_vsyscall */
	return seq;
}

static notrace int vdso_read_retry(const struct vdso_data *vd, u32 start)
{
	u32 seq;

	smp_rmb(); /* Pairs with first smp_wmb in update_vsyscall */
	seq = READ_ONCE(vd->tb_seq_count);
	return seq != start;
}

static notrace long clock_gettime_fallback(clockid_t _clkid,
					   struct timespec *_ts)
{
	register struct timespec *ts asm("r1") = _ts;
	register clockid_t clkid asm("r0") = _clkid;
	register long ret asm ("r0");
	register long nr asm("r7") = __NR_clock_gettime;

	asm volatile(
	"	swi #0\n"
	: "=r" (ret)
	: "r" (clkid), "r" (ts), "r" (nr)
	: "memory");

	return ret;
}

static notrace int do_realtime_coarse(const struct vdso_data *vd,
				      struct timespec *ts)
{
	u32 seq;

	do {
		seq = vdso_read_begin(vd);

		ts->tv_sec = vd->xtime_coarse_sec;
		ts->tv_nsec = vd->xtime_coarse_nsec;

	} while (vdso_read_retry(vd, seq));

	return 0;
}

static notrace int do_monotonic_coarse(const struct vdso_data *vd,
				       struct timespec *ts)
{
	struct timespec tomono;
	u32 seq;
	u64 nsec;

	do {
		seq = vdso_read_begin(vd);

		ts->tv_sec = vd->xtime_coarse_sec;
		ts->tv_nsec = vd->xtime_coarse_nsec;

		tomono.tv_sec = vd->wtm_clock_sec;
		tomono.tv_nsec = vd->wtm_clock_nsec;

	} while (vdso_read_retry(vd, seq));

	ts->tv_sec += tomono.tv_sec;
	/* open coding timespec_add_ns */
	ts->tv_sec += __iter_div_u64_rem(ts->tv_nsec + tomono.tv_nsec,
					 NSEC_PER_SEC, &nsec);
	ts->tv_nsec = nsec;

	return 0;
}

#ifdef CONFIG_ARM_ARCH_TIMER

/*
 * Returns the clock delta, in nanoseconds left-shifted by the clock
 * shift.
 */
static notrace u64 get_clock_shifted_nsec(const u64 cycle_last,
					  const u32 mult,
					  const u64 mask)
{
	u64 res;

	isb();
<<<<<<< HEAD
	cycle_now = read_sysreg(CNTVCT);

	cycle_delta = (cycle_now - vd->cs_cycle_last) & vd->cs_mask;
=======
	/* Read the virtual counter. */
	res = arch_vdso_read_counter();
>>>>>>> a790acc4b21a... FROMLIST: [PATCH v5 04/12] arm: vdso: do calculations outside reader loops

	res = res - cycle_last;

	res &= mask;
	return res * mult;
}

static notrace int do_realtime(const struct vdso_data *vd, struct timespec *ts)
{
	u32 seq, mult, shift;
	u64 nsec, cycle_last;
	u64 mask;
	vdso_xtime_clock_sec_t sec;

	do {
		seq = vdso_read_begin(vd);

		if (vd->use_syscall)
			return -1;

		cycle_last = vd->cs_cycle_last;

		mult = vd->cs_mono_mult;
		shift = vd->cs_shift;
		mask = vd->cs_mask;

		sec = vd->xtime_clock_sec;
		nsec = vd->xtime_clock_snsec;

	} while (unlikely(vdso_read_retry(vd, seq)));

	nsec += get_clock_shifted_nsec(cycle_last, mult, mask);
	nsec >>= shift;
	/* open coding timespec_add_ns to save a ts->tv_nsec = 0 */
	ts->tv_sec = sec + __iter_div_u64_rem(nsec, NSEC_PER_SEC, &nsec);
	ts->tv_nsec = nsec;

	return 0;
}

static notrace int do_monotonic(const struct vdso_data *vd, struct timespec *ts)
{
	u32 seq, mult, shift;
	u64 nsec, cycle_last;
	u64 mask;
	vdso_wtm_clock_nsec_t wtm_nsec;
	__kernel_time_t sec;

	do {
		seq = vdso_read_begin(vd);

		if (vd->use_syscall)
			return -1;

		cycle_last = vd->cs_cycle_last;

		mult = vd->cs_mono_mult;
		shift = vd->cs_shift;
		mask = vd->cs_mask;

		sec = vd->xtime_clock_sec;
		nsec = vd->xtime_clock_snsec;

		sec += vd->wtm_clock_sec;
		wtm_nsec = vd->wtm_clock_nsec;

	} while (unlikely(vdso_read_retry(vd, seq)));

	nsec += get_clock_shifted_nsec(cycle_last, mult, mask);
	nsec >>= shift;
	nsec += wtm_nsec;
	/* open coding timespec_add_ns to save a ts->tv_nsec = 0 */
	ts->tv_sec = sec + __iter_div_u64_rem(nsec, NSEC_PER_SEC, &nsec);
	ts->tv_nsec = nsec;

	return 0;
}

#else /* CONFIG_ARM_ARCH_TIMER */

static notrace int do_realtime(const struct vdso_data *vd, struct timespec *ts)
{
	return -1;
}

static notrace int do_monotonic(const struct vdso_data *vd, struct timespec *ts)
{
	return -1;
}

#endif /* CONFIG_ARM_ARCH_TIMER */

notrace int __vdso_clock_gettime(clockid_t clkid, struct timespec *ts)
{
	int ret = -1;

	const struct vdso_data *vd = __get_datapage();

	switch (clkid) {
	case CLOCK_REALTIME_COARSE:
		ret = do_realtime_coarse(vd, ts);
		break;
	case CLOCK_MONOTONIC_COARSE:
		ret = do_monotonic_coarse(vd, ts);
		break;
	case CLOCK_REALTIME:
		ret = do_realtime(vd, ts);
		break;
	case CLOCK_MONOTONIC:
		ret = do_monotonic(vd, ts);
		break;
	default:
		break;
	}

	if (ret)
		ret = clock_gettime_fallback(clkid, ts);

	return ret;
}

static notrace long gettimeofday_fallback(struct timeval *_tv,
					  struct timezone *_tz)
{
	register struct timezone *tz asm("r1") = _tz;
	register struct timeval *tv asm("r0") = _tv;
	register long ret asm ("r0");
	register long nr asm("r7") = __NR_gettimeofday;

	asm volatile(
	"	swi #0\n"
	: "=r" (ret)
	: "r" (tv), "r" (tz), "r" (nr)
	: "memory");

	return ret;
}

notrace int __vdso_gettimeofday(struct timeval *tv, struct timezone *tz)
{
	struct timespec ts;
	int ret;

	const struct vdso_data *vd = __get_datapage();

	ret = do_realtime(vd, &ts);
	if (ret)
		return gettimeofday_fallback(tv, tz);

	if (tv) {
		tv->tv_sec = ts.tv_sec;
		tv->tv_usec = ts.tv_nsec / 1000;
	}
	if (tz) {
		tz->tz_minuteswest = vd->tz_minuteswest;
		tz->tz_dsttime = vd->tz_dsttime;
	}

	return ret;
}

/* Avoid unresolved references emitted by GCC */

void __aeabi_unwind_cpp_pr0(void)
{
}

void __aeabi_unwind_cpp_pr1(void)
{
}

void __aeabi_unwind_cpp_pr2(void)
{
}
