#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0xe4c970fb, "module_layout" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x14c42ae8, "kmem_cache_destroy" },
	{ 0x60e2055d, "cdev_del" },
	{ 0xe94c7a3a, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{  0x22a98, "cdev_init" },
	{ 0x7d91149b, "pci_enable_sriov" },
	{ 0xa4b4fd0a, "__put_devmap_managed_page" },
	{ 0xdc086bbc, "genl_register_family" },
	{ 0x5516a5ea, "debugfs_create_dir" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0x91eb9b4, "round_jiffies" },
	{ 0x754d539c, "strlen" },
	{ 0xcfe5bf38, "genl_unregister_family" },
	{ 0xd6e0ab14, "dma_set_mask" },
	{ 0x184558c, "pcie_set_readrq" },
	{ 0x80daf046, "pci_disable_device" },
	{ 0x45aa56e9, "pci_disable_msix" },
	{ 0xb3635b01, "_raw_spin_lock_bh" },
	{ 0xa97dbfb8, "pci_disable_sriov" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x837b7b09, "__dynamic_pr_debug" },
	{ 0x260bbe45, "device_destroy" },
	{ 0xc29957c3, "__x86_indirect_thunk_rcx" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x3e375e47, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xebf8864b, "pcie_capability_clear_and_set_word" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0x85df9b6c, "strsep" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x167e7f9d, "__get_user_1" },
	{ 0x82f607bf, "pcie_get_readrq" },
	{ 0x9f04a881, "dma_free_attrs" },
	{ 0x69972cd2, "debugfs_create_file" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0xbab17439, "debugfs_remove_recursive" },
	{ 0x61422f4f, "sysfs_remove_group" },
	{ 0xc1221f92, "__alloc_pages_nodemask" },
	{ 0x300deb20, "pv_ops" },
	{ 0x6b8d427a, "set_page_dirty" },
	{ 0x844809c5, "dma_set_coherent_mask" },
	{ 0x61a395f5, "kthread_create_on_node" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x1972d5cd, "param_ops_string" },
	{ 0x95a26215, "kthread_bind" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xb44ad4b3, "_copy_to_user" },
	{ 0xe0ee1ec4, "pci_set_master" },
	{ 0x3c80c06c, "kstrtoull" },
	{ 0x7e526bfa, "__x86_indirect_thunk_r10" },
	{ 0xfb578fc5, "memset" },
	{ 0x37b0990c, "pci_restore_state" },
	{ 0x50d9a1f9, "pci_iounmap" },
	{ 0x963dcba1, "_raw_spin_trylock_bh" },
	{ 0x3812050a, "_raw_spin_unlock_irqrestore" },
	{ 0x977f511b, "__mutex_init" },
	{ 0xc5850110, "printk" },
	{ 0xbcab6ee6, "sscanf" },
	{ 0xb05e8293, "kthread_stop" },
	{ 0x689f749b, "sysfs_create_group" },
	{ 0xa1c76e0a, "_cond_resched" },
	{ 0x9166fada, "strncpy" },
	{ 0x6e3cd53f, "nla_put" },
	{ 0xe8c97f94, "dma_direct_map_page" },
	{ 0xe12fe387, "dma_alloc_attrs" },
	{ 0xa3d09b76, "kmem_cache_free" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0x241c4110, "device_create" },
	{ 0xc38c83b8, "mod_timer" },
	{ 0x79664ae7, "netlink_unicast" },
	{ 0x2072ee9b, "request_threaded_irq" },
	{ 0x2276db98, "kstrtoint" },
	{ 0x3356b6da, "_dev_err" },
	{ 0x42160169, "flush_workqueue" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x47cb61f7, "cdev_add" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0xa263ceb0, "dma_direct_unmap_page" },
	{ 0xe552de3, "_dev_info" },
	{ 0x931b1d3, "kmem_cache_alloc" },
	{ 0xf40ef106, "__free_pages" },
	{ 0xb601be4c, "__x86_indirect_thunk_rdx" },
	{ 0x618911fc, "numa_node" },
	{ 0x9f4b4bc0, "__alloc_skb" },
	{ 0xa916b694, "strnlen" },
	{ 0x714ba584, "pci_enable_msix_range" },
	{ 0xfe916dc6, "hex_dump_to_buffer" },
	{ 0x49c41a57, "_raw_spin_unlock_bh" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x7114f4f4, "pci_cleanup_aer_uncorrect_error_status" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0xe2bb5c52, "kfree_skb" },
	{ 0x2ea2c95c, "__x86_indirect_thunk_rax" },
	{ 0xaf036f7b, "wake_up_process" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x6446db14, "pci_unregister_driver" },
	{ 0xc622a0d1, "kmem_cache_alloc_trace" },
	{ 0xdbf17652, "_raw_spin_lock" },
	{ 0xb19a5453, "__per_cpu_offset" },
	{ 0x51760917, "_raw_spin_lock_irqsave" },
	{ 0x8cad971, "kmem_cache_create" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xdc0e4855, "timer_delete" },
	{ 0xc5f38f48, "pci_sriov_get_totalvfs" },
	{ 0x37a0cba, "kfree" },
	{ 0x69ad2f20, "kstrtouint" },
	{ 0x69acdf38, "memcpy" },
	{ 0xa8435fc3, "genlmsg_put" },
	{ 0xb6f41547, "pci_request_regions" },
	{ 0x6df1aaf1, "kernel_sigaction" },
	{ 0xba467990, "pci_num_vf" },
	{ 0x2f02ac15, "__pci_register_driver" },
	{ 0x96848186, "scnprintf" },
	{ 0x201d0f70, "class_destroy" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x1ba59527, "__kmalloc_node" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xe8027e90, "pci_vfs_assigned" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x7b17fe1f, "pci_iomap" },
	{ 0xbcec9c9c, "pci_enable_device_mem" },
	{ 0xb423141a, "pci_bus_max_busnr" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0xc60d0620, "__num_online_cpus" },
	{ 0x4eb16ccf, "pci_enable_device" },
	{ 0xdc9a426d, "pci_msix_vec_count" },
	{ 0xd1aa9ce7, "param_ops_uint" },
	{ 0xa640d12, "__class_create" },
	{ 0xdf9208c0, "alloc_workqueue" },
	{ 0x9e7d6bd0, "__udelay" },
	{ 0xb61f0c7c, "dma_ops" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0xb2a650f1, "__put_page" },
	{ 0x3086af15, "get_user_pages_fast" },
	{ 0xc1514a3b, "free_irq" },
	{ 0xd2921aac, "pci_save_state" },
	{ 0xe914e41e, "strcpy" },
	{ 0x587f22d7, "devmap_managed_key" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("pci:v000010EEd00009011sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009111sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009211sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009311sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009012sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009112sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009212sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009312sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009114sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009214sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009314sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009118sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009218sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009318sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000901Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000911Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000921Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000931Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009021sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009121sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009221sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009321sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009022sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009122sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009222sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009322sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009024sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009124sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009224sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009324sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009028sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009128sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009228sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009328sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000902Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000912Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000922Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000932Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009031sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009131sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009231sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009331sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009032sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009132sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009232sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009332sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009034sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009134sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009234sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009334sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009038sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009138sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009238sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009338sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000903Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000913Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000923Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000933Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00006AA0sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009041sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009141sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009241sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009341sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009042sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009142sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009242sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009342sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009044sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009144sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009244sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009344sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009048sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009148sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009248sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd00009348sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B011sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B111sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B211sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B311sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B012sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B112sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B212sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B312sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B114sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B214sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B314sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B118sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B218sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B318sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B01Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B11Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B21Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B31Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B021sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B121sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B221sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B321sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B022sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B122sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B222sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B322sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B024sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B124sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B224sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B324sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B028sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B128sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B228sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B328sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B02Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B12Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B22Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B32Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B031sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B131sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B231sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B331sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B032sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B132sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B232sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B332sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B034sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B134sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B234sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B334sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B038sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B138sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B238sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B338sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B03Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B13Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B23Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B33Fsv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B041sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B141sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B241sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B341sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B042sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B142sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B242sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B342sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B044sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B144sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B244sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B344sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B048sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B148sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B248sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B348sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B058sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B158sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B258sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v000010EEd0000B358sv*sd*bc*sc*i*");

MODULE_INFO(srcversion, "48B6D98D512520288A7FCC2");
