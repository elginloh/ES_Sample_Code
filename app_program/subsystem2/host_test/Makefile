################################################################################
# Host-side tests of the pure-logic modules of all five subsystems.
#
# NOT part of the firmware: the firmware makefile only compiles
# app_program/*.c and app_program/*/*.c, never this folder.  The same test
# functions (sN_unit_tests.c, sN_integration_tests.c) also run on the Pico
# in the "make TEST=1" image.
#
#   make            build and run every suite
#   make SUITES="s3 int"   run selected suites (s1..s5, int)
#   make OUT=/tmp/x run    put the binary somewhere else (default ./build)
#
# shim/ replaces <tk/typedef.h>, <tk/errno.h> and <tm/tmonitor.h> with
# exact-width host versions (T-Kernel's W/UW are 'long', 64-bit on a host).
################################################################################

CC	?= cc
APP	:= ../..
OUT	?= ./build
SUITES	?=

CFLAGS	:= -std=gnu11 -O1 -g -Wall -Wextra -Wno-unused-parameter \
	   -DHOST_TEST=1 -DAPP_TEST=0 -Ishim

SRCS := host_main.c \
	$(APP)/subsystem1/bitpack.c \
	$(APP)/subsystem1/mqtt_proto.c \
	$(APP)/subsystem1/comm_fsm.c \
	$(APP)/subsystem1/tlm_build.c \
	$(APP)/subsystem1/s1_unit_tests.c \
	$(APP)/subsystem1/s1_integration_tests.c \
	$(APP)/subsystem2/sys_state_logic.c \
	$(APP)/subsystem2/motion_cmd.c \
	$(APP)/subsystem2/pid.c \
	$(APP)/subsystem2/motion_ctrl.c \
	$(APP)/subsystem2/vehicle_fsm.c \
	$(APP)/subsystem2/s2_unit_tests.c \
	$(APP)/subsystem2/s2_integration_tests.c \
	$(APP)/subsystem3/code39.c \
	$(APP)/subsystem3/barcode_dec.c \
	$(APP)/subsystem3/nav_map.c \
	$(APP)/subsystem3/line_follow.c \
	$(APP)/subsystem3/s3_status.c \
	$(APP)/subsystem3/s3_unit_tests.c \
	$(APP)/subsystem4/imu_proc.c \
	$(APP)/subsystem4/hump.c \
	$(APP)/subsystem4/s4_status.c \
	$(APP)/subsystem4/s4_unit_tests.c \
	$(APP)/subsystem5/s5_msg.c \
	$(APP)/subsystem5/obstacle_profile.c \
	$(APP)/subsystem5/avoid_plan.c \
	$(APP)/subsystem5/s5_unit_tests.c

.PHONY: all run clean
all: run

$(OUT)/host_tests: $(SRCS) $(wildcard $(APP)/*/*.h) $(wildcard shim/*/*.h)
	@mkdir -p $(OUT)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

run: $(OUT)/host_tests
	$(OUT)/host_tests $(SUITES)

clean:
	rm -rf $(OUT)
