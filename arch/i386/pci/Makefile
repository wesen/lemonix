obj-y				:= i386.o init.o

obj-$(CONFIG_PCI_BIOS)		+= pcbios.o
obj-$(CONFIG_PCI_MMCONFIG)	+= mmconfig.o direct.o mmconfig-shared.o
obj-$(CONFIG_PCI_DIRECT)	+= direct.o

obj-$(CONFIG_ACPI)		+= acpi.o

pci-y				:= fixup.o
pci-y				+= legacy.o irq.o

pci-$(CONFIG_X86_VISWS)		:= visws.o fixup.o
pci-$(CONFIG_X86_NUMAQ)		:= numa.o irq.o

obj-y				+= $(pci-y) common.o early.o
