#
# Makefile for Kernel-based Virtual Machine module
#
EXTRA_CFLAGS :=

kvm-objs := kvm_main.o mmu.o x86_emulate.o kvm_net_host.o kvm_block.o
obj-$(CONFIG_KVM) += kvm.o
kvm-intel-objs = vmx.o
obj-$(CONFIG_KVM_INTEL) += kvm-intel.o
kvm-amd-objs = svm.o
obj-$(CONFIG_KVM_AMD) += kvm-amd.o
kvm-net-objs = kvm_net.o
obj-$(CONFIG_KVM_NET) += kvm-net.o
