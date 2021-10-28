/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef SPO2_ALGORITHM_H_
#define SPO2_ALGORITHM_H_
    
#include <stdint.h>
#include <stdbool.h>

#define FreqS 25    //sampling frequency
#define BUFFER_SIZE (FreqS * 4) 
#define MA4_SIZE 4 // DONOT CHANGE


//uch_spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
extern const uint8_t uch_spo2_table[184];


void maxim_heart_rate_and_oxygen_saturation(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, 
                int32_t *pn_heart_rate, int8_t *pch_hr_valid);


void maxim_find_peaks(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);
void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height);
void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance);
void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t  *pn_x, int32_t *pn_indx, int32_t n_size);

#endif /* SPO2_ALGORITHM_H_ */

/* [] END OF FILE */
