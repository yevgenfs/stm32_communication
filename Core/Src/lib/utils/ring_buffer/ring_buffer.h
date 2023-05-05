/*
 * ring_buffer.h
 *
 *  Created on: 5 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#ifndef SRC_LIB_UTILS_RING_BUFFER_RING_BUFFER_H_
#define SRC_LIB_UTILS_RING_BUFFER_RING_BUFFER_H_

#include "stdint.h"

/// @brief queue_err_t enum which express queue erorrs
typedef enum
{
    e_ring_buffer_err_ok,
    e_ring_buffer_err_que_is_full,
    e_ring_buffer_err_que_is_empty,
    e_ring_buffer_err_invalid_argument,
    e_ring_buffer_err_allocation_error,
} ring_buffer_err_t;

/// @brief ring_buffer_t struct which express queue of spinner_ctrl_t
typedef struct
{
    uint32_t        u32_rear;
    uint32_t        u32_front;
    uint32_t        u32_size;
    uint32_t        u32_curent_size;
    uint8_t*        P_array;
}ring_buffer_t;

/**
 @brief function which create ring buffer

 @param[in][out] ring_buffer_this instance of ring buffer  which should create

 @param[in] size  of ring buffer which should create

 @return return type of error or ok if work correctly
 */
ring_buffer_err_t create_ring_buffer(ring_buffer_t *ring_buffer_this, uint32_t size);

/**
 @brief function which add spinner_ctrl_t in  ring buffer

 @param[in][out] ring_buffer_this instance of ring buffer in which should add element

 @param[in] P_item instance of uint8_t which add to the ring buffer

 @return return type of error or ok if work correctly
 */
ring_buffer_err_t en_ring_buffer(ring_buffer_t *ring_buffer_this, uint8_t *P_item);

/**
 @brief function which return spinner_ctrl_t from  ring buffer

 @param[in] ring_buffer_this instance of ring buffer from which should return element

 @param[out] P_item instance of uint8_t which return from the queue

 @return return type of error or ok if work correctly
 */
ring_buffer_err_t de_ring_buffer(ring_buffer_t *ring_buffer_this, uint8_t *P_item);

/**
 @brief function which return curent_size ring buffer

 @param[in] ring_buffer_this instance of ring buffer from which should return size

 @return return uint32_t u32_size
 */
uint32_t get_curent_size_ring_buffer (ring_buffer_t *ring_buffer_this);

/**
 @brief function which delete ring buffer

 @param[in] ring_buffer_this instance of ring buffer  which should delete

 @return return type of error or ok if work correctly
 */
ring_buffer_err_t delete_ring_buffer (ring_buffer_t *ring_buffer_this);

#endif /* SRC_LIB_UTILS_RING_BUFFER_RING_BUFFER_H_ */
