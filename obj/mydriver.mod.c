#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x092a35a2, "_copy_from_user" },
	{ 0xcb8b6ec6, "kfree" },
	{ 0x86d206f6, "__SCT__WARN_trap" },
	{ 0x092a35a2, "_copy_to_user" },
	{ 0xb6b7f2c0, "devm_kmalloc" },
	{ 0xd9324140, "cdev_init" },
	{ 0x9f222e1e, "alloc_chrdev_region" },
	{ 0x45d300cd, "cdev_add" },
	{ 0x85be535a, "device_create" },
	{ 0xdf9cca1e, "device_create_file" },
	{ 0x5e6d74fc, "platform_device_unregister" },
	{ 0xb620cc7d, "platform_driver_unregister" },
	{ 0xd7442be0, "class_destroy" },
	{ 0xbd03ed67, "__ref_stack_chk_guard" },
	{ 0xfad798b2, "class_create" },
	{ 0xdc54cc96, "__platform_driver_register" },
	{ 0xac5fb72f, "platform_device_register_full" },
	{ 0xd272d446, "__stack_chk_fail" },
	{ 0xd272d446, "__fentry__" },
	{ 0xd272d446, "__x86_return_thunk" },
	{ 0xe8213e80, "_printk" },
	{ 0x529b94b4, "device_remove_file" },
	{ 0x0f889972, "device_destroy" },
	{ 0x73634a54, "cdev_del" },
	{ 0x0bc5fb0d, "unregister_chrdev_region" },
	{ 0xdd6830c7, "sysfs_emit" },
	{ 0xfbe7861b, "memcpy" },
	{ 0xd710adbf, "__kmalloc_noprof" },
	{ 0x5cb46e6d, "validate_usercopy_range" },
	{ 0xa61fd7aa, "__check_object_size" },
	{ 0xe9196a28, "module_layout" },
};

static const u32 ____version_ext_crcs[]
__used __section("__version_ext_crcs") = {
	0x092a35a2,
	0xcb8b6ec6,
	0x86d206f6,
	0x092a35a2,
	0xb6b7f2c0,
	0xd9324140,
	0x9f222e1e,
	0x45d300cd,
	0x85be535a,
	0xdf9cca1e,
	0x5e6d74fc,
	0xb620cc7d,
	0xd7442be0,
	0xbd03ed67,
	0xfad798b2,
	0xdc54cc96,
	0xac5fb72f,
	0xd272d446,
	0xd272d446,
	0xd272d446,
	0xe8213e80,
	0x529b94b4,
	0x0f889972,
	0x73634a54,
	0x0bc5fb0d,
	0xdd6830c7,
	0xfbe7861b,
	0xd710adbf,
	0x5cb46e6d,
	0xa61fd7aa,
	0xe9196a28,
};
static const char ____version_ext_names[]
__used __section("__version_ext_names") =
	"_copy_from_user\0"
	"kfree\0"
	"__SCT__WARN_trap\0"
	"_copy_to_user\0"
	"devm_kmalloc\0"
	"cdev_init\0"
	"alloc_chrdev_region\0"
	"cdev_add\0"
	"device_create\0"
	"device_create_file\0"
	"platform_device_unregister\0"
	"platform_driver_unregister\0"
	"class_destroy\0"
	"__ref_stack_chk_guard\0"
	"class_create\0"
	"__platform_driver_register\0"
	"platform_device_register_full\0"
	"__stack_chk_fail\0"
	"__fentry__\0"
	"__x86_return_thunk\0"
	"_printk\0"
	"device_remove_file\0"
	"device_destroy\0"
	"cdev_del\0"
	"unregister_chrdev_region\0"
	"sysfs_emit\0"
	"memcpy\0"
	"__kmalloc_noprof\0"
	"validate_usercopy_range\0"
	"__check_object_size\0"
	"module_layout\0"
;

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Cfabricante,modulo");
MODULE_ALIAS("of:N*T*Cfabricante,moduloC*");

MODULE_INFO(srcversion, "40E2DC3EC8799E1EAF9EA29");
