#ifndef V4L2_LINK_FREQ
#define V4L2_LINK_FREQ

/**
 * v4l2_link_freq_to_bitmap - Figure out platform-supported link frequencies
 * @dev: The struct device
 * @fw_link_freqs: Array of link frequencies from firmware
 * @num_of_fw_link_freqs: Number of entries in @fw_link_freqs
 * @driver_link_freqs: Array of link frequencies supported by the driver
 * @num_of_driver_link_freqs: Number of entries in @driver_link_freqs
 * @bitmap: Bitmap of driver-supported link frequencies found in @fw_link_freqs
 *
 * This function checks which driver-supported link frequencies are enabled in
 * system firmware and sets the corresponding bits in @bitmap (after first
 * zeroing it).
 *
 * Return values:
 *      0: Success
 *      -ENOENT: No match found between driver-supported link frequencies and
 *               those available in firmware.
 *      -ENODATA: No link frequencies were specified in firmware.
 */
int v4l2_link_freq_to_bitmap(struct device *dev, const u64 *fw_link_freqs,
                             unsigned int num_of_fw_link_freqs,
                             const s64 *driver_link_freqs,
                             unsigned int num_of_driver_link_freqs,
                             unsigned long *bitmap);

#endif
