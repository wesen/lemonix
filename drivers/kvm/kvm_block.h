#ifndef __INCLUDE_GUARD_KVMBLOCK_H
#define __INCLUDE_GUARD_KVMBLOCK_H

struct kvm_vector {
        uint64_t        data_gpa;
        uint64_t        data_len;
};

struct kvm_iopage {
	uint64_t        sector;
        uint64_t        guest_data;
        uint32_t        sector_count;
        uint32_t        write;
        uint32_t        entries;
        uint32_t        ioresult;
        struct kvm_vector vector[126];
};

struct kvm_disk_info {
	uint64_t        sectors;
	uint64_t	reserved[15];
};

#endif
