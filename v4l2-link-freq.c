// SPDX-License-Identifier: GPL-2.0
/*
 * backport module of v4l2_link_freq_to_bitmap
 */


#include <linux/module.h>
#include <linux/device.h>

int v4l2_link_freq_to_bitmap(struct device *dev, const u64 *fw_link_freqs,
			     unsigned int num_of_fw_link_freqs,
			     const s64 *driver_link_freqs,
			     unsigned int num_of_driver_link_freqs,
			     unsigned long *bitmap)
{
	unsigned int i;

	*bitmap = 0;

	if (!num_of_fw_link_freqs) {
		dev_err(dev, "no link frequencies in firmware\n");
		return -ENODATA;
	}

	for (i = 0; i < num_of_fw_link_freqs; i++) {
		unsigned int j;

		for (j = 0; j < num_of_driver_link_freqs; j++) {
			if (fw_link_freqs[i] != driver_link_freqs[j])
				continue;

			dev_dbg(dev, "enabling link frequency %lld Hz\n",
				driver_link_freqs[j]);
			*bitmap |= BIT(j);
			break;
		}
	}

	if (!*bitmap) {
		dev_err(dev, "no matching link frequencies found\n");

		dev_dbg(dev, "specified in firmware:\n");
		for (i = 0; i < num_of_fw_link_freqs; i++)
			dev_dbg(dev, "\t%llu Hz\n", fw_link_freqs[i]);

		dev_dbg(dev, "driver supported:\n");
		for (i = 0; i < num_of_driver_link_freqs; i++)
			dev_dbg(dev, "\t%lld Hz\n", driver_link_freqs[i]);

		return -ENOENT;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(v4l2_link_freq_to_bitmap);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("v4l2_link_freq_to_bitmap backport");
