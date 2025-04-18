/* auto-generated by gen_syscalls.py, don't edit */

#ifndef Z_INCLUDE_SYSCALLS_FUEL_GAUGE_H
#define Z_INCLUDE_SYSCALLS_FUEL_GAUGE_H


#include <zephyr/tracing/tracing_syscall.h>

#ifndef _ASMLANGUAGE

#include <stdarg.h>

#include <zephyr/syscall_list.h>
#include <zephyr/syscall.h>

#include <zephyr/linker/sections.h>


#ifdef __cplusplus
extern "C" {
#endif

extern int z_impl_fuel_gauge_get_prop(const struct device * dev, fuel_gauge_prop_t prop, union fuel_gauge_prop_val * val);

__pinned_func
static inline int fuel_gauge_get_prop(const struct device * dev, fuel_gauge_prop_t prop, union fuel_gauge_prop_val * val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; fuel_gauge_prop_t val; } parm1 = { .val = prop };
		union { uintptr_t x; union fuel_gauge_prop_val * val; } parm2 = { .val = val };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_FUEL_GAUGE_GET_PROP);
	}
#endif
	compiler_barrier();
	return z_impl_fuel_gauge_get_prop(dev, prop, val);
}

#if defined(CONFIG_TRACING_SYSCALL)
#ifndef DISABLE_SYSCALL_TRACING

#define fuel_gauge_get_prop(dev, prop, val) ({ 	int syscall__retval; 	sys_port_trace_syscall_enter(K_SYSCALL_FUEL_GAUGE_GET_PROP, fuel_gauge_get_prop, dev, prop, val); 	syscall__retval = fuel_gauge_get_prop(dev, prop, val); 	sys_port_trace_syscall_exit(K_SYSCALL_FUEL_GAUGE_GET_PROP, fuel_gauge_get_prop, dev, prop, val, syscall__retval); 	syscall__retval; })
#endif
#endif


extern int z_impl_fuel_gauge_get_props(const struct device * dev, const fuel_gauge_prop_t * props, union fuel_gauge_prop_val * vals, size_t len);

__pinned_func
static inline int fuel_gauge_get_props(const struct device * dev, const fuel_gauge_prop_t * props, union fuel_gauge_prop_val * vals, size_t len)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; const fuel_gauge_prop_t * val; } parm1 = { .val = props };
		union { uintptr_t x; union fuel_gauge_prop_val * val; } parm2 = { .val = vals };
		union { uintptr_t x; size_t val; } parm3 = { .val = len };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_FUEL_GAUGE_GET_PROPS);
	}
#endif
	compiler_barrier();
	return z_impl_fuel_gauge_get_props(dev, props, vals, len);
}

#if defined(CONFIG_TRACING_SYSCALL)
#ifndef DISABLE_SYSCALL_TRACING

#define fuel_gauge_get_props(dev, props, vals, len) ({ 	int syscall__retval; 	sys_port_trace_syscall_enter(K_SYSCALL_FUEL_GAUGE_GET_PROPS, fuel_gauge_get_props, dev, props, vals, len); 	syscall__retval = fuel_gauge_get_props(dev, props, vals, len); 	sys_port_trace_syscall_exit(K_SYSCALL_FUEL_GAUGE_GET_PROPS, fuel_gauge_get_props, dev, props, vals, len, syscall__retval); 	syscall__retval; })
#endif
#endif


extern int z_impl_fuel_gauge_set_prop(const struct device * dev, fuel_gauge_prop_t prop, union fuel_gauge_prop_val val);

__pinned_func
static inline int fuel_gauge_set_prop(const struct device * dev, fuel_gauge_prop_t prop, union fuel_gauge_prop_val val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; fuel_gauge_prop_t val; } parm1 = { .val = prop };
		union { uintptr_t x; union fuel_gauge_prop_val val; } parm2 = { .val = val };
		return (int) arch_syscall_invoke3(parm0.x, parm1.x, parm2.x, K_SYSCALL_FUEL_GAUGE_SET_PROP);
	}
#endif
	compiler_barrier();
	return z_impl_fuel_gauge_set_prop(dev, prop, val);
}

#if defined(CONFIG_TRACING_SYSCALL)
#ifndef DISABLE_SYSCALL_TRACING

#define fuel_gauge_set_prop(dev, prop, val) ({ 	int syscall__retval; 	sys_port_trace_syscall_enter(K_SYSCALL_FUEL_GAUGE_SET_PROP, fuel_gauge_set_prop, dev, prop, val); 	syscall__retval = fuel_gauge_set_prop(dev, prop, val); 	sys_port_trace_syscall_exit(K_SYSCALL_FUEL_GAUGE_SET_PROP, fuel_gauge_set_prop, dev, prop, val, syscall__retval); 	syscall__retval; })
#endif
#endif


extern int z_impl_fuel_gauge_set_props(const struct device * dev, const fuel_gauge_prop_t * props, const union fuel_gauge_prop_val * vals, size_t len);

__pinned_func
static inline int fuel_gauge_set_props(const struct device * dev, const fuel_gauge_prop_t * props, const union fuel_gauge_prop_val * vals, size_t len)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; const fuel_gauge_prop_t * val; } parm1 = { .val = props };
		union { uintptr_t x; const union fuel_gauge_prop_val * val; } parm2 = { .val = vals };
		union { uintptr_t x; size_t val; } parm3 = { .val = len };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_FUEL_GAUGE_SET_PROPS);
	}
#endif
	compiler_barrier();
	return z_impl_fuel_gauge_set_props(dev, props, vals, len);
}

#if defined(CONFIG_TRACING_SYSCALL)
#ifndef DISABLE_SYSCALL_TRACING

#define fuel_gauge_set_props(dev, props, vals, len) ({ 	int syscall__retval; 	sys_port_trace_syscall_enter(K_SYSCALL_FUEL_GAUGE_SET_PROPS, fuel_gauge_set_props, dev, props, vals, len); 	syscall__retval = fuel_gauge_set_props(dev, props, vals, len); 	sys_port_trace_syscall_exit(K_SYSCALL_FUEL_GAUGE_SET_PROPS, fuel_gauge_set_props, dev, props, vals, len, syscall__retval); 	syscall__retval; })
#endif
#endif


extern int z_impl_fuel_gauge_get_buffer_prop(const struct device * dev, fuel_gauge_prop_t prop_type, void * dst, size_t dst_len);

__pinned_func
static inline int fuel_gauge_get_buffer_prop(const struct device * dev, fuel_gauge_prop_t prop_type, void * dst, size_t dst_len)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		union { uintptr_t x; fuel_gauge_prop_t val; } parm1 = { .val = prop_type };
		union { uintptr_t x; void * val; } parm2 = { .val = dst };
		union { uintptr_t x; size_t val; } parm3 = { .val = dst_len };
		return (int) arch_syscall_invoke4(parm0.x, parm1.x, parm2.x, parm3.x, K_SYSCALL_FUEL_GAUGE_GET_BUFFER_PROP);
	}
#endif
	compiler_barrier();
	return z_impl_fuel_gauge_get_buffer_prop(dev, prop_type, dst, dst_len);
}

#if defined(CONFIG_TRACING_SYSCALL)
#ifndef DISABLE_SYSCALL_TRACING

#define fuel_gauge_get_buffer_prop(dev, prop_type, dst, dst_len) ({ 	int syscall__retval; 	sys_port_trace_syscall_enter(K_SYSCALL_FUEL_GAUGE_GET_BUFFER_PROP, fuel_gauge_get_buffer_prop, dev, prop_type, dst, dst_len); 	syscall__retval = fuel_gauge_get_buffer_prop(dev, prop_type, dst, dst_len); 	sys_port_trace_syscall_exit(K_SYSCALL_FUEL_GAUGE_GET_BUFFER_PROP, fuel_gauge_get_buffer_prop, dev, prop_type, dst, dst_len, syscall__retval); 	syscall__retval; })
#endif
#endif


extern int z_impl_fuel_gauge_battery_cutoff(const struct device * dev);

__pinned_func
static inline int fuel_gauge_battery_cutoff(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (int) arch_syscall_invoke1(parm0.x, K_SYSCALL_FUEL_GAUGE_BATTERY_CUTOFF);
	}
#endif
	compiler_barrier();
	return z_impl_fuel_gauge_battery_cutoff(dev);
}

#if defined(CONFIG_TRACING_SYSCALL)
#ifndef DISABLE_SYSCALL_TRACING

#define fuel_gauge_battery_cutoff(dev) ({ 	int syscall__retval; 	sys_port_trace_syscall_enter(K_SYSCALL_FUEL_GAUGE_BATTERY_CUTOFF, fuel_gauge_battery_cutoff, dev); 	syscall__retval = fuel_gauge_battery_cutoff(dev); 	sys_port_trace_syscall_exit(K_SYSCALL_FUEL_GAUGE_BATTERY_CUTOFF, fuel_gauge_battery_cutoff, dev, syscall__retval); 	syscall__retval; })
#endif
#endif


#ifdef __cplusplus
}
#endif

#endif
#endif /* include guard */
